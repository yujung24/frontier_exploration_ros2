#!/usr/bin/env python3

import math
import time
from types import SimpleNamespace

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener

from travel_time.continuous_timing import (
    InvalidMotionState,
    InvalidTimingPath,
    connect_start,
    validate_motion_state,
)
from travel_time.continuous_timing_v3 import evaluate_path


TIME_COST = {"A": "time_distance_v3_s", "B": "time_turn_v3_s", "C": "time_accel_v3_s"}
ENDPOINT_TOLERANCE_M = 0.20


def yaw_of(q):
    length = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(length) or length == 0.0:
        return math.nan
    x, y, z, w = (value / length for value in (q.x, q.y, q.z, q.w))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class TravelTimeSelector(Node):
    def __init__(self):
        super().__init__("travel_time_selector")
        self.frontier_list = None
        self.current_goal_active = False
        self.grid = None
        self.odometry = None
        self.batch = None
        self.retry_at = 0.0

        strategy = self.declare_parameter("time_strategy", "C").value
        if strategy not in TIME_COST:
            raise ValueError("time_strategy must be A, B or C")
        self.cost_key = TIME_COST[strategy]
        self.planner_id = self.declare_parameter("planner_id", "GridBased").value
        self.min_goal_distance = float(self.declare_parameter("frontier_selection_min_distance", 0.5).value)

        self.create_subscription(PoseArray, "/frontier_list", self.frontiers_callback, 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(Odometry, "/odom", self.odometry_callback, qos_profile_sensor_data)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.selection_timer = self.create_timer(
            0.05,
            self.update_selection,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def map_callback(self, msg):
        if msg.header.frame_id != "map":
            self.grid = None
            return
        info = msg.info
        self.grid = SimpleNamespace(
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            data=tuple(msg.data),
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            origin_yaw=yaw_of(info.origin.orientation),
        )

    def odometry_callback(self, msg):
        self.odometry = msg

    def get_robot_pose(self, body_frame):
        try:
            transform = self.tf_buffer.lookup_transform("map", body_frame, rclpy.time.Time())
        except TransformException:
            return None
        position = transform.transform.translation
        return position.x, position.y, yaw_of(transform.transform.rotation), transform

    def motion_snapshot(self):
        msg = self.odometry
        if msg is None or msg.header.frame_id != "odom":
            return None
        if msg.child_frame_id not in ("base_link", "base_footprint"):
            return None
        pose = self.get_robot_pose(msg.child_frame_id)
        if pose is None:
            return None
        x, y, yaw, transform = pose
        now = self.get_clock().now().nanoseconds
        odom_stamp = stamp_ns(msg.header.stamp)
        pose_stamp = stamp_ns(transform.header.stamp)
        if (
            min(odom_stamp, pose_stamp) <= 0
            or not 0 <= now - odom_stamp <= 500_000_000
            or not 0 <= now - pose_stamp <= 1_000_000_000
            or abs(odom_stamp - pose_stamp) > 500_000_000
        ):
            return None
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        if not all(map(math.isfinite, (x, y, yaw, v.x, v.y, v.z, w.x, w.y, w.z))):
            return None
        if math.hypot(v.y, v.z) > 0.01 or math.hypot(w.x, w.y) > 0.01:
            return None
        try:
            validate_motion_state(v.x, w.z)
        except InvalidMotionState:
            return None
        return SimpleNamespace(xy=(x, y), yaw=yaw, v=v.x, w=w.z)

    def goal_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def frontiers_callback(self, msg):
        if msg.header.frame_id != "map" or self.current_goal_active:
            return
        self.frontier_list = tuple((p.position.x, p.position.y) for p in msg.poses)

    def update_selection(self):
        now = time.monotonic()
        if self.batch is not None:
            self.poll_plan(now)
            return
        if self.current_goal_active or not self.frontier_list or now < self.retry_at:
            return
        if self.grid is None or not self.planner.server_is_ready() or not self.nav_client.server_is_ready():
            return
        motion = self.motion_snapshot()
        if motion is None:
            return
        candidates = [
            point for point in self.frontier_list
            if math.isfinite(point[0]) and math.isfinite(point[1])
            and math.dist(motion.xy, point) >= self.min_goal_distance
        ]
        if not candidates:
            return
        self.batch = SimpleNamespace(
            frontiers=self.frontier_list,
            candidates=iter(candidates),
            motion=motion,
            grid=self.grid,
            deadline=now + 8.0,
            request=None,
            result=None,
            handle=None,
            point=None,
            best_point=None,
            best_cost=math.inf,
        )
        self.request_path()

    def request_path(self):
        batch = self.batch
        now = time.monotonic()
        if now >= batch.deadline:
            self.finish_batch()
            return
        point = next(batch.candidates, None)
        if point is None:
            self.finish_batch()
            return
        batch.point = point
        request = ComputePathToPose.Goal()
        request.start = PoseStamped()
        request.start.header.frame_id = "map"
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose.position.x = batch.motion.xy[0]
        request.start.pose.position.y = batch.motion.xy[1]
        request.start.pose.orientation.w = math.cos(batch.motion.yaw / 2.0)
        request.start.pose.orientation.z = math.sin(batch.motion.yaw / 2.0)
        request.goal = self.goal_pose(*point)
        request.use_start = True
        request.planner_id = self.planner_id
        batch.request_deadline = min(batch.deadline, now + 1.0)
        batch.expired = False
        batch.cancel_sent = False
        batch.request = self.planner.send_goal_async(request)
        batch.handle = None
        batch.result = None

    def poll_plan(self, now):
        batch = self.batch
        if not batch.expired and now >= batch.request_deadline:
            batch.expired = True
        if batch.handle is None:
            if not batch.request.done():
                if batch.expired and not batch.cancel_sent:
                    batch.cancel_sent = True
                    # Sending cancellation requires an accepted handle, retried below.
                return
            handle = batch.request.result()
            if not handle.accepted:
                self.request_path()
                return
            batch.handle = handle
            batch.result = handle.get_result_async()
        if not batch.result.done():
            if batch.expired and not batch.cancel_sent:
                batch.cancel_sent = True
                batch.handle.cancel_goal_async()
            return
        response = batch.result.result()
        if not batch.expired and response.status == GoalStatus.STATUS_SUCCEEDED:
            self.score_frontier(response.result.path)
        self.request_path()

    def score_frontier(self, path):
        batch = self.batch
        if path.header.frame_id != "map":
            return
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        if not points or math.dist(points[-1], batch.point) > ENDPOINT_TOLERANCE_M:
            return
        try:
            points, connector = connect_start(points, batch.motion.xy, batch.grid, max_deviation_m=ENDPOINT_TOLERANCE_M)
            speed, _ = validate_motion_state(batch.motion.v, batch.motion.w)
            if connector["clearance_basis"] == "bounded_planner_start_proxy" and speed > 0.0:
                return
            cost = evaluate_path(
                points,
                batch.motion.yaw,
                0.0,
                batch.grid,
                batch.motion.v,
                batch.motion.w,
                goal_xy=batch.point,
            )[self.cost_key]
        except (InvalidMotionState, InvalidTimingPath):
            return
        if math.isfinite(cost) and cost < batch.best_cost:
            batch.best_point, batch.best_cost = batch.point, cost

    def finish_batch(self):
        batch = self.batch
        self.batch = None
        self.retry_at = time.monotonic() + 0.5
        if batch.best_point is None:
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.goal_pose(*batch.best_point)
        self.current_goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.current_goal_active = False
            return
        handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            status = future.result().status
        except Exception:
            status = None
        self.current_goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = TravelTimeSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
