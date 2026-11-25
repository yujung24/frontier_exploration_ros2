#!/usr/bin/env python3

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Empty

from frontier_exploration_ros2.srv import ControlExploration


def parse_pgm(path):
    """Read an ASCII or binary PGM as a 2-D uint8 array."""
    with open(path, "rb") as f:
        raw = f.read()

    pos = 0

    def read_header_token(buf, pos):
        while pos < len(buf):
            if chr(buf[pos]).isspace():
                pos += 1
            elif buf[pos] == 35:
                while pos < len(buf) and buf[pos] != 10:
                    pos += 1
            else:
                break
        start = pos
        while pos < len(buf) and not chr(buf[pos]).isspace() and buf[pos] != 35:
            pos += 1
        if start == pos:
            raise ValueError(f"Invalid PGM header in {path}")
        return buf[start:pos], pos

    magic, pos = read_header_token(raw, pos)
    if magic not in (b"P5", b"P2"):
        raise ValueError(f"Unsupported PGM format in {path}")

    width_bytes, pos = read_header_token(raw, pos)
    height_bytes, pos = read_header_token(raw, pos)
    maxval_bytes, pos = read_header_token(raw, pos)
    width = int(width_bytes)
    height = int(height_bytes)
    maxval = int(maxval_bytes)

    if magic == b"P2":
        pixels = []
        while len(pixels) < width * height:
            token, pos = read_header_token(raw, pos)
            pixels.append(int(token))
        if len(pixels) != width * height:
            raise ValueError(f"Invalid PGM dimensions in {path}")
        pixels = np.asarray(pixels, dtype=np.float32)
    else:
        # P5 raster starts after exactly one whitespace separator following maxval.
        if pos < len(raw) and chr(raw[pos]).isspace():
            pos += 1
        raster = raw[pos:pos + width * height]
        if len(raster) != width * height:
            raise ValueError(f"Truncated PGM raster: {path}")
        pixels = np.frombuffer(raster, dtype=np.uint8).astype(np.float32)

    pixels = pixels.reshape((height, width))
    if maxval > 0:
        pixels = pixels * (255.0 / float(maxval))
    return pixels


def load_reference(path, args):
    path = Path(path).expanduser().resolve()
    resolution = args.resolution
    origin_x = args.origin_x
    origin_y = args.origin_y
    origin_yaw = args.origin_yaw
    free_thresh = args.reference_free_threshold
    negate = False

    if path.suffix.lower() in (".yaml", ".yml"):
        with path.open("r", encoding="utf-8") as f:
            metadata = yaml.safe_load(f)
        image_name = metadata["image"]
        image_path = (path.parent / image_name).resolve()
        resolution = float(metadata.get("resolution", resolution if resolution is not None else 0.05))
        origin = metadata.get("origin", [origin_x or 0.0, origin_y or 0.0, origin_yaw or 0.0])
        origin_x, origin_y, origin_yaw = map(float, origin)
        negate = bool(metadata.get("negate", negate))
        free_thresh = float(metadata.get("free_thresh", free_thresh))
    else:
        image_path = path

    suffix = image_path.suffix.lower()
    if suffix == ".pgm":
        pixels = parse_pgm(str(image_path))
    else:
        try:
            from PIL import Image
        except ImportError as exc:
            raise RuntimeError("Direct reference-map images require Pillow") from exc
        with Image.open(image_path) as image:
            pixels = np.asarray(image.convert("L"), dtype=np.float32)

    if suffix == ".pgm" or path.suffix.lower() in (".yaml", ".yml"):
        # Negate is the ROS occupancy-map convention.
        if negate:
            occupied_pct = pixels * 100.0 / 255.0
        else:
            occupied_pct = (255.0 - pixels) * 100.0 / 255.0
        floor = occupied_pct <= (free_thresh * 100.0)
    else:
        # A floor mask image: bright pixels are floor; dark pixels are not.
        floor = pixels >= args.reference_floor_pixel

    if resolution is None:
        raise ValueError("Reference map resolution is required (YAML resolution or --resolution)")

    return floor.astype(bool), float(resolution), float(origin_x), float(origin_y), float(origin_yaw)


def quaternion_yaw(z, w):
    return 2.0 * math.atan2(z, w)


def map_world_to_index(map_msg, world_x, world_y):
    info = map_msg.info
    ox = info.origin.position.x
    oy = info.origin.position.y
    yaw = quaternion_yaw(info.origin.orientation.z, info.origin.orientation.w)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx = world_x - ox
    dy = world_y - oy
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    ix = int(math.floor(local_x / info.resolution))
    iy = int(math.floor(local_y / info.resolution))
    return ix, iy


class CoverageMonitor(Node):
    def __init__(self, args):
        super().__init__("coverage_monitor")
        self.args = args
        self.threshold = float(args.target_coverage)
        self.reference_floor, self.ref_resolution, self.ref_ox, self.ref_oy, self.ref_yaw = load_reference(
            args.reference_map, args
        )
        self.total_cells = int(np.count_nonzero(self.reference_floor))
        self.latest_explored = 0
        self.latest_coverage = 0.0
        self.finished = False
        self.stop_requested = False
        self.last_progress_log_wall = 0.0

        self.coverage_reached_pub = self.create_publisher(Empty, args.coverage_reached_topic, 10)
        self.create_subscription(OccupancyGrid, args.map_topic, self.map_callback, 10)
        self.create_subscription(Empty, args.completion_topic, self.completion_callback, 10)
        self.stop_client = self.create_client(ControlExploration, args.service_name)

        height, width = self.reference_floor.shape
        self.get_logger().info(
            f"Reference map loaded: {width}x{height}, resolution={self.ref_resolution:.6f} m, "
            f"evaluation cells={self.total_cells}, target={self.threshold:.1%}"
        )

    def map_callback(self, msg):
        if self.finished:
            return

        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or self.total_cells <= 0:
            return

        slam = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        slam_free = (slam >= 0) & (slam <= int(self.args.free_threshold))

        slam_free_on_ref = np.zeros(self.reference_floor.shape, dtype=bool)
        cos_yaw = math.cos(self.ref_yaw)
        sin_yaw = math.sin(self.ref_yaw)

        ref_height, ref_width = self.reference_floor.shape
        for ref_y in range(ref_height):
            local_y = (ref_y + 0.5) * self.ref_resolution
            for ref_x in range(ref_width):
                if not self.reference_floor[ref_y, ref_x]:
                    continue
                local_x = (ref_x + 0.5) * self.ref_resolution
                wx = self.ref_ox + cos_yaw * local_x - sin_yaw * local_y
                wy = self.ref_oy + sin_yaw * local_x + cos_yaw * local_y
                ix, iy = map_world_to_index(msg, wx, wy)
                if 0 <= ix < width and 0 <= iy < height:
                    slam_free_on_ref[ref_y, ref_x] = bool(slam_free[iy, ix])

        total_cells = np.count_nonzero(self.reference_floor)
        explored_cells = np.count_nonzero(self.reference_floor & slam_free_on_ref)
        coverage = explored_cells / total_cells if total_cells > 0 else 0.0
        self.latest_explored = int(explored_cells)
        self.latest_coverage = float(coverage)

        self.get_logger().debug(
            f"Coverage: {coverage:.4%} ({explored_cells}/{total_cells})"
        )

        wall_now = time.time()
        if wall_now - self.last_progress_log_wall >= float(self.args.progress_log_interval):
            self.last_progress_log_wall = wall_now
            self.get_logger().info(
                f"Exploration progress: {coverage:.2%} ({explored_cells}/{total_cells} cells), target={self.threshold:.0%}"
            )

        if 10 * explored_cells >= 9 * total_cells and coverage + 1e-12 >= self.threshold:
            now = self.get_clock().now().nanoseconds / 1e9
            wall_time = datetime.now(timezone.utc).isoformat()
            self.finished = True
            self.write_result("target_reached", now, wall_time)
            self.get_logger().info(
                f"Target reached: {coverage:.2%} ({explored_cells}/{total_cells}) at {wall_time}"
            )
            self.coverage_reached_pub.publish(Empty())
            if self.args.auto_stop:
                self.request_stop()

    def completion_callback(self, _msg):
        if self.finished:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        wall_time = datetime.now(timezone.utc).isoformat()
        self.finished = True
        self.write_result("frontier_exhausted", now, wall_time)
        self.get_logger().warn(
            f"Exploration complete at {self.latest_coverage:.2%} "
            f"({self.latest_explored}/{self.total_cells}); target {self.threshold:.2%} not reached"
        )

    def request_stop(self):
        if self.stop_requested:
            return
        self.stop_requested = True
        if not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Stop service '{self.args.service_name}' is not available"
            )
            return

        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self.stop_client.call_async(request)

        def stopped(response_future):
            try:
                response = response_future.result()
                if response.accepted:
                    self.get_logger().info(f"Stop accepted: {response.message}")
                else:
                    self.get_logger().error(f"Stop rejected: {response.message}")
            except Exception as exc:  # pylint: disable=broad-exception-caught
                self.get_logger().error(f"Stop request failed: {exc}")

        future.add_done_callback(stopped)

    def write_result(self, reason, ros_time, wall_time):
        result = {
            "reason": reason,
            "target_coverage": self.threshold,
            "coverage": self.latest_coverage,
            "explored_cells": self.latest_explored,
            "total_cells": self.total_cells,
            "first_reached_at_ros_seconds": ros_time,
            "wall_time": wall_time,
        }
        path = Path(self.args.output).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        self.get_logger().info(f"Result written to {path}")


def build_parser():
    parser = argparse.ArgumentParser(description="Stop exploration at 90% coverage of a reference floor map")
    parser.add_argument("--reference-map", "--reference_map", "--reference", "-r", required=True)
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--completion-topic", default="exploration_complete")
    parser.add_argument("--service-name", "--service", default="control_exploration")
    parser.add_argument("--target-coverage", "--target", type=float, default=0.90)
    parser.add_argument("--free-threshold", type=int, default=25,
                        help="SLAM map values <= this value are free (default: 25)")
    parser.add_argument("--reference-free-threshold", type=float, default=0.25,
                        help="Free threshold for reference occupancy-map YAML (0.0 to 1.0)")
    parser.add_argument("--reference-floor-pixel", type=int, default=128,
                        help="Pixels >= this value are floor in direct PNG/JPG masks")
    parser.add_argument("--resolution", type=float, default=None)
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-yaw", type=float, default=0.0)
    parser.add_argument("--output", default="coverage_result.json")
    parser.add_argument("--coverage-reached-topic", default="coverage_target_reached")
    parser.add_argument("--auto-stop", action="store_true", default=True,
                        help="Also call the frontier control stop service (default: enabled)")
    parser.add_argument("--no-auto-stop", dest="auto_stop", action="store_false",
                        help="Only publish the coverage event; let Mission Manager stop exploration")
    parser.add_argument("--progress-log-interval", type=float, default=5.0,
                        help="Seconds between INFO-level live coverage progress logs")
    return parser


def main():
    parser = build_parser()
    args, ros_args = parser.parse_known_args()
    if not 0.0 < args.target_coverage <= 1.0:
        parser.error("--target-coverage must be in (0, 1]")
    if args.progress_log_interval <= 0.0:
        parser.error("--progress-log-interval must be greater than 0")

    rclpy.init(args=ros_args)
    node = CoverageMonitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
