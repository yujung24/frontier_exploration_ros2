#!/usr/bin/env python3

import json
import math
import os
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_exploration_ros2.srv import ControlExploration


class MissionManager(Node):
    EXPLORING = "EXPLORING"
    EXPLORATION_COMPLETE = "EXPLORATION_COMPLETE"
    CANCELING_FRONTIER_GOAL = "CANCELING_FRONTIER_GOAL"
    RETURNING_TO_START = "RETURNING_TO_START"
    PLANNING_GLOBAL_PATH = "PLANNING_GLOBAL_PATH"
    NAVIGATING_TO_FINAL_GOAL = "NAVIGATING_TO_FINAL_GOAL"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_FAILED = "MISSION_FAILED"

    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("mission_config", "")
        self.declare_parameter("world_name", "default")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("planner_id", "GridBased")
        self.declare_parameter("control_exploration_service", "control_exploration")
        self.declare_parameter("coverage_reached_topic", "coverage_target_reached")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("output", "mission_result.json")
        self.declare_parameter("goal_timeout_s", 300.0)
        self.declare_parameter("cancel_timeout_s", 10.0)
        self.declare_parameter("collision_distance_m", 0.12)

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("robot_base_frame").value
        self.output = os.path.abspath(os.path.expanduser(self.get_parameter("output").value))
        self.goal_timeout = float(self.get_parameter("goal_timeout_s").value)
        self.cancel_timeout = float(self.get_parameter("cancel_timeout_s").value)
        self.collision_distance = float(self.get_parameter("collision_distance_m").value)

        world_name = self.get_parameter("world_name").value
        config_path = self.get_parameter("mission_config").value
        self.configured_start, self.final_pose = self.load_route(config_path, world_name)
        self.planner_id = self.get_parameter("planner_id").value

        self.state = self.EXPLORING
        self.started_wall = time.time()
        self.started_ros = self.get_clock().now()
        self.start_pose = self.configured_start
        self.active_handle = None
        self.active_phase = "exploration"
        self.phase_started_wall = None
        self.phase_started_position = None
        self.last_position = None
        self.total_distance = 0.0
        self.exploration_distance = 0.0
        self.return_distance = 0.0
        self.final_navigation_distance = 0.0
        self.collision_count = 0
        self.minimum_obstacle_distance = float("inf")
        self.stop_future = None
        self.stop_deadline = None

        self.result = {
            "state": self.state,
            "world_name": world_name,
            "exploration": {"coverage_90_reached": False},
            "navigation": {},
            "mission": {},
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_to_pose_action").value,
        )
        self.planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(NavPath, "mission/global_path", path_qos)
        self.trajectory_pub = self.create_publisher(NavPath, "mission/actual_path", path_qos)
        self.actual_path = NavPath()
        self.actual_path.header.frame_id = self.global_frame
        self.stop_client = self.create_client(
            ControlExploration,
            self.get_parameter("control_exploration_service").value,
        )

        coverage_topic = self.get_parameter("coverage_reached_topic").value
        self.create_subscription(Empty, coverage_topic, self.coverage_reached, 10)
        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        scan_topic = self.get_parameter("scan_topic").value
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 20)
        self.timeout_timer = self.create_timer(
            0.5, self.timeout_callback, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        if self.start_pose is None:
            self.capture_start_pose()
        else:
            self.result["start_pose"] = self.start_pose
        self.get_logger().info(
            f"Mission Manager ready; state={self.state}, final={self.final_pose['position']}"
        )

    def load_route(self, config_path, world_name):
        if config_path:
            path = Path(config_path).expanduser().resolve()
        else:
            path = Path(__file__).resolve().parent.parent / "config" / "mission.yaml"

        if not path.exists():
            raise RuntimeError(f"Mission config not found: {path}")
        with path.open("r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}

        worlds = config.get("worlds", {})
        selected = worlds.get(world_name) or worlds.get("default")
        if not selected:
            raise RuntimeError(f"No mission destination for world '{world_name}' in {path}")

        start = selected.get("start")
        return (
            self.read_pose(start) if start is not None else None,
            self.read_pose(selected.get("goal", selected)),
        )

    @staticmethod
    def read_pose(selected):
        position = selected.get("position", selected)
        if "yaw" in selected:
            yaw = float(selected["yaw"])
            orientation = {"z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}
        else:
            orientation = selected.get("orientation", {})
        return {
            "position": {"x": float(position["x"]), "y": float(position["y"]), "z": float(position.get("z", 0.0))},
            "orientation": {"x": float(orientation.get("x", 0.0)), "y": float(orientation.get("y", 0.0)),
                            "z": float(orientation.get("z", 0.0)), "w": float(orientation.get("w", 1.0))},
        }

    def capture_start_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
            self.start_pose = {
                "position": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                },
                "orientation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w,
                },
                "frame": self.global_frame,
            }
            self.result["start_pose"] = self.start_pose
            self.get_logger().info("Captured exploration start pose in map frame")
        except TransformException as exc:
            self.get_logger().warn(f"Start pose TF not ready: {exc}")
            self.create_timer(0.5, self.retry_start_pose)

    def retry_start_pose(self):
        if self.start_pose is not None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
            self.start_pose = {
                "position": {"x": transform.transform.translation.x,
                             "y": transform.transform.translation.y,
                             "z": transform.transform.translation.z},
                "orientation": {"x": transform.transform.rotation.x,
                                "y": transform.transform.rotation.y,
                                "z": transform.transform.rotation.z,
                                "w": transform.transform.rotation.w},
                "frame": self.global_frame,
            }
            self.result["start_pose"] = self.start_pose
        except TransformException:
            pass

    def odom_callback(self, msg):
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_position is not None:
            dx = point[0] - self.last_position[0]
            dy = point[1] - self.last_position[1]
            distance = math.hypot(dx, dy)
            if distance < 1.0:
                self.total_distance += distance
                if self.active_phase == "exploration":
                    self.exploration_distance += distance
                elif self.active_phase == "return":
                    self.return_distance += distance
                elif self.active_phase == "final":
                    self.final_navigation_distance += distance
        self.last_position = point

    def scan_callback(self, msg):
        valid = [d for d in msg.ranges if math.isfinite(d) and d > 0.02]
        if not valid:
            return
        minimum = min(valid)
        self.minimum_obstacle_distance = min(self.minimum_obstacle_distance, minimum)
        if minimum < self.collision_distance:
            self.collision_count += 1

    def coverage_reached(self, _msg):
        if self.state != self.EXPLORING:
            return
        if self.start_pose is None:
            self.fail("Start pose was unavailable when coverage 90% was reached")
            return
        self.state = self.EXPLORATION_COMPLETE
        self.result["exploration"]["coverage_90_reached"] = True
        self.result["exploration"]["exploration_reached_wall_time"] = time.time()
        self.result["exploration"]["exploration_mission_distance_m"] = self.exploration_distance
        self.active_phase = None
        self.transition_to_canceling()

    def transition_to_canceling(self):
        self.state = self.CANCELING_FRONTIER_GOAL
        self.result["state"] = self.state
        self.get_logger().info("Stopping Frontier Explorer and canceling active navigation goal")

        if not self.stop_client.wait_for_service(timeout_sec=2.0):
            self.fail("control_exploration service is unavailable")
            return

        self.stop_deadline = time.monotonic() + self.cancel_timeout
        self.request_explorer_stop()

    def request_explorer_stop(self):
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        self.stop_future = self.stop_client.call_async(request)
        self.stop_future.add_done_callback(self.stop_response_callback)

    def stop_response_callback(self, future):
        self.stop_future = None
        if self.state != self.CANCELING_FRONTIER_GOAL:
            return
        try:
            response = future.result()
            if not response.accepted:
                self.fail(f"Frontier stop rejected: {response.message}")
                return
        except Exception as exc:
            self.fail(f"Frontier stop service failed: {exc}")
            return

        if response.state == ControlExploration.Request.STATE_STOPPING:
            return
        if response.state != ControlExploration.Request.STATE_IDLE:
            self.fail(f"Unexpected Frontier Explorer state after stop: {response.state}")
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.fail("NavigateToPose action server is unavailable")
            return

        self.stop_deadline = None
        self.result["navigation"]["cancel"] = {"explorer_idle": True}
        self.get_logger().info("Frontier Explorer is idle; navigating to the starting pose")
        self.begin_return()

    def begin_return(self):
        self.state = self.RETURNING_TO_START
        self.result["state"] = self.state
        self.active_phase = "return"
        self.phase_started_wall = time.time()
        self.result["navigation"]["return"] = {
            "start_wall_time": self.phase_started_wall,
            "start_pose": self.start_pose,
        }
        self.send_goal(self.start_pose, "return")

    def begin_global_plan(self):
        self.state = self.PLANNING_GLOBAL_PATH
        self.result["state"] = self.state
        self.active_phase = "plan"
        self.phase_started_wall = time.time()
        self.result["navigation"]["global_plan"] = {"planner_id": self.planner_id}
        if not self.planner.wait_for_server(timeout_sec=2.0):
            self.fail("ComputePathToPose action server is unavailable")
            return
        request = ComputePathToPose.Goal()
        request.start = self.pose_stamped(self.start_pose)
        request.goal = self.pose_stamped(self.final_pose)
        request.use_start = True
        request.planner_id = self.planner_id
        self.get_logger().info(f"Planning global path with {self.planner_id}")
        self.planner.send_goal_async(request).add_done_callback(
            lambda future: self.goal_response(future, "plan")
        )

    def global_plan_result(self, wrapped):
        path = wrapped.result.path
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.fail(f"Global planning returned no usable path (status={wrapped.status})")
            return
        positions = [pose.pose.position for pose in path.poses]
        length = sum(
            math.hypot(second.x - first.x, second.y - first.y)
            for first, second in zip(positions, positions[1:])
        )
        duration = wrapped.result.planning_time
        self.result["navigation"]["global_plan"].update({
            "success": True,
            "path_length_m": length,
            "pose_count": len(path.poses),
            "planning_time_s": duration.sec + duration.nanosec / 1e9,
            "path_xy": [[point.x, point.y] for point in positions],
        })
        self.path_pub.publish(path)
        self.get_logger().info(f"Global path ready: {len(path.poses)} poses, {length:.2f} m")
        self.begin_final_goal()

    def begin_final_goal(self):
        self.state = self.NAVIGATING_TO_FINAL_GOAL
        self.result["state"] = self.state
        self.active_phase = "final"
        self.phase_started_wall = time.time()
        self.result["navigation"]["final"] = {
            "start_wall_time": self.phase_started_wall,
            "goal": self.final_pose,
        }
        self.sample_final_pose()
        self.get_logger().info("Navigating to the final destination")
        self.send_goal(self.final_pose, "final")

    def pose_stamped(self, pose):
        stamped = PoseStamped()
        stamped.header.frame_id = self.global_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = pose["position"]["x"]
        stamped.pose.position.y = pose["position"]["y"]
        stamped.pose.position.z = pose["position"]["z"]
        stamped.pose.orientation.x = pose["orientation"]["x"]
        stamped.pose.orientation.y = pose["orientation"]["y"]
        stamped.pose.orientation.z = pose["orientation"]["z"]
        stamped.pose.orientation.w = pose["orientation"]["w"]
        return stamped

    def send_goal(self, pose, phase):
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(pose)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda handle_future: self.goal_response(handle_future, phase))

    def goal_response(self, future, phase):
        try:
            handle = future.result()
        except Exception as exc:
            if phase != self.active_phase:
                return
            self.fail(f"{phase} goal submission failed: {exc}")
            return
        if phase != self.active_phase:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail(f"Nav2 rejected {phase} goal")
            return
        self.active_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result_future: self.goal_result(result_future, phase)
        )

    def goal_result(self, future, phase):
        if phase != self.active_phase:
            return
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            self.fail(f"{phase} goal result failed: {exc}")
            return

        self.active_handle = None
        if phase == "plan":
            self.global_plan_result(wrapped)
            return

        elapsed = time.time() - (self.phase_started_wall or time.time())
        section = self.result["navigation"][phase]
        section["elapsed_s"] = elapsed

        if status == GoalStatus.STATUS_SUCCEEDED:
            section["success"] = True
            section["success_wall_time"] = time.time()
            if phase == "return":
                section["actual_distance_m"] = self.return_distance
                self.begin_global_plan()
            else:
                self.sample_final_pose()
                section["actual_distance_m"] = self.final_navigation_distance
                section["trajectory_xy"] = [
                    [sample.pose.position.x, sample.pose.position.y]
                    for sample in self.actual_path.poses
                ]
                self.complete_mission()
        elif status == GoalStatus.STATUS_CANCELED:
            section["success"] = False
            section["status"] = "canceled"
            self.fail(f"{phase} navigation was canceled")
        else:
            section["success"] = False
            section["status"] = "aborted"
            self.fail(f"{phase} navigation aborted")

    def sample_final_pose(self):
        try:
            sample = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return
        pose = PoseStamped()
        pose.header = sample.header
        pose.pose.position.x = sample.transform.translation.x
        pose.pose.position.y = sample.transform.translation.y
        pose.pose.position.z = sample.transform.translation.z
        pose.pose.orientation = sample.transform.rotation
        self.actual_path.header.stamp = pose.header.stamp
        self.actual_path.poses.append(pose)
        self.trajectory_pub.publish(self.actual_path)

    def timeout_callback(self):
        if self.state == self.CANCELING_FRONTIER_GOAL:
            if time.monotonic() >= self.stop_deadline:
                self.fail("Timed out waiting for Frontier Explorer to become idle")
            elif self.stop_future is None:
                self.request_explorer_stop()
            return
        if self.active_phase is not None and self.phase_started_wall is not None:
            if time.time() - self.phase_started_wall > self.goal_timeout:
                self.fail(f"Navigation timeout while in state {self.state}")
            elif self.active_phase == "final":
                self.sample_final_pose()

    def fail(self, message):
        if self.state == self.MISSION_FAILED:
            return
        self.state = self.MISSION_FAILED
        self.active_phase = None
        self.result["state"] = self.state
        self.result["error"] = message
        self.get_logger().error(message)
        if self.active_handle is not None:
            try:
                self.active_handle.cancel_goal_async()
            except Exception:
                pass
        self.write_result()

    def complete_mission(self):
        self.state = self.MISSION_COMPLETE
        self.active_phase = None
        self.result["state"] = self.state
        elapsed = time.time() - self.started_wall
        self.result["exploration"].update({
            "total_exploration_and_navigation_distance_m": self.total_distance,
            "minimum_obstacle_distance_m": None if math.isinf(self.minimum_obstacle_distance) else self.minimum_obstacle_distance,
            "collision_count": self.collision_count,
        })
        self.result["mission"] = {
            "success": True,
            "elapsed_s": elapsed,
            "total_distance_m": self.total_distance,
            "wall_complete_time": time.time(),
        }
        self.get_logger().info("Mission complete")
        self.write_result()

    def write_result(self):
        if self.state != self.MISSION_COMPLETE:
            self.result.setdefault("mission", {})
            self.result["mission"].update({
                "success": False,
                "elapsed_s": time.time() - self.started_wall,
                "total_distance_m": self.total_distance,
            })
        Path(self.output).parent.mkdir(parents=True, exist_ok=True)
        with open(self.output, "w", encoding="utf-8") as f:
            json.dump(self.result, f, indent=2)
        self.get_logger().info(f"Mission result written to {self.output}")


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
