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
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_exploration_ros2.srv import ControlExploration


class MissionManager(Node):
    EXPLORING = "EXPLORING"
    EXPLORATION_COMPLETE = "EXPLORATION_COMPLETE"
    CANCELING_FRONTIER_GOAL = "CANCELING_FRONTIER_GOAL"
    RETURNING_TO_START = "RETURNING_TO_START"
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
        self.final_pose = self.load_final_pose(config_path, world_name)

        self.state = self.EXPLORING
        self.started_wall = time.time()
        self.started_ros = self.get_clock().now()
        self.start_pose = None
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
        self.phase_cancellations = 0

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
        self.timeout_timer = self.create_timer(0.5, self.timeout_callback)

        self.capture_start_pose()
        self.get_logger().info(
            f"Mission Manager ready; state={self.state}, final={self.final_pose['position']}"
        )

    def load_final_pose(self, config_path, world_name):
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

        position = selected["position"]
        orientation = selected.get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
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

        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self.stop_client.call_async(request)
        future.add_done_callback(self.stop_response_callback)

    def stop_response_callback(self, future):
        try:
            response = future.result()
            if not response.accepted:
                self.fail(f"Frontier stop rejected: {response.message}")
                return
        except Exception as exc:
            self.fail(f"Frontier stop service failed: {exc}")
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.fail("NavigateToPose action server is unavailable")
            return

        if self.active_handle is not None:
            cancel_future = self.active_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
        else:
            try:
                cancel_all_future = self.nav_client._client.cancel_all_goals_async()
                cancel_all_future.add_done_callback(self.cancel_response_callback)
            except Exception as exc:
                self.fail(f"Failed to request Nav2 goal cancellation: {exc}")

    def cancel_response_callback(self, future):
        try:
            response = future.result()
            accepted = response is not None
        except Exception:
            accepted = False
        if not accepted:
            self.fail("Nav2 did not accept cancellation of the Frontier goal")
            return
        self.phase_cancellations += 1
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

    def begin_final_goal(self):
        self.state = self.NAVIGATING_TO_FINAL_GOAL
        self.result["state"] = self.state
        self.active_phase = "final"
        self.phase_started_wall = time.time()
        self.result["navigation"]["final"] = {
            "start_wall_time": self.phase_started_wall,
            "goal": self.final_pose,
        }
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
            self.fail(f"{phase} goal submission failed: {exc}")
            return
        if not handle.accepted:
            self.fail(f"Nav2 rejected {phase} goal")
            return
        self.active_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result_future: self.goal_result(result_future, phase)
        )

    def goal_result(self, future, phase):
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            self.fail(f"{phase} goal result failed: {exc}")
            return

        elapsed = time.time() - (self.phase_started_wall or time.time())
        section = self.result["navigation"][phase]
        section["elapsed_s"] = elapsed

        if status == GoalStatus.STATUS_SUCCEEDED:
            section["success"] = True
            section["success_wall_time"] = time.time()
            self.active_handle = None
            if phase == "return":
                section["actual_distance_m"] = self.return_distance
                self.begin_final_goal()
            else:
                section["actual_distance_m"] = self.final_navigation_distance
                self.complete_mission()
        elif status == GoalStatus.STATUS_CANCELED:
            section["success"] = False
            section["status"] = "canceled"
            self.fail(f"{phase} navigation was canceled")
        else:
            section["success"] = False
            section["status"] = "aborted"
            self.fail(f"{phase} navigation aborted")

    def timeout_callback(self):
        if self.active_phase is not None and self.phase_started_wall is not None:
            if time.time() - self.phase_started_wall > self.goal_timeout:
                self.fail(f"Navigation timeout while in state {self.state}")

    def fail(self, message):
        if self.state == self.MISSION_FAILED:
            return
        self.state = self.MISSION_FAILED
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
        self.result["navigation"].setdefault("cancel", {})
        self.result["navigation"]["cancel"]["accepted_count"] = self.phase_cancellations
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
