#!/usr/bin/env python3

import argparse
import math
import sys
import zlib
from collections import Counter, deque
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


KARTO_MINIMUM_TIME_INTERVAL_ASSUMED = 3600.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def bool_string(value: bool) -> str:
    return "yes" if value else "no"


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float

    def squared_distance(self, other: "Pose2D") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return dx * dx + dy * dy


class SlamScanGateDiagnostics(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("slam_scan_gate_diagnostics")
        self.args = args

        self.tf_buffer = Buffer()
        self.tf_topic = args.tf_topic
        self.tf_static_topic = args.tf_static_topic
        self.tf_message_count = 0
        self.tf_static_message_count = 0
        self.tf_sub = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self.tf_callback,
            QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.tf_static_sub = self.create_subscription(
            TFMessage,
            self.tf_static_topic,
            self.tf_static_callback,
            QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.cfg = {
            "odom_frame": "odom",
            "base_frame": "base_footprint",
            "throttle_scans": 1,
            "minimum_time_interval": 0.2,
            "minimum_travel_distance": 0.0,
            "minimum_travel_heading": 0.1,
            "min_laser_range": 0.0,
            "max_laser_range": 25.0,
            "map_update_interval": 0.5,
            "paused_new_measurements": False,
        }
        self.cfg_source = "defaults"
        self._param_future = None
        self._param_client = None
        self._setup_param_client()

        self.total_scans = 0
        self.layer1_reject = Counter()
        self.layer2_reject = Counter()
        self.layer3_reject = Counter()
        self.scan_frame_counter = Counter()
        self.layer1_debug_counter = Counter()
        self.layer1_samples = deque(maxlen=8)
        self.layer2_accept = 0
        self.layer3_accept = 0

        self.outer_last_pose: Optional[Pose2D] = None
        self.outer_last_stamp: Optional[Time] = None
        self.outer_first_measurement = True
        self.outer_scan_ctr = 0

        self.karto_last_sensor_pose: Optional[Pose2D] = None
        self.karto_last_stamp: Optional[Time] = None
        self.karto_first_scan = True

        self.beam_total = 0
        self.beam_ignored = 0
        self.beam_clipped = 0
        self.beam_endpoint_hit = 0
        self.accepted_scans_zero_usable_beams = 0
        self.accepted_scans_zero_endpoint_hits = 0

        self.map_publish_count = 0
        self.map_change_count = 0
        self.last_map_hash: Optional[int] = None
        self.last_map_stamp: Optional[Time] = None
        self.last_map_change_stamp: Optional[Time] = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            args.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            args.map_topic,
            self.map_callback,
            map_qos,
        )

        self.report_timer = self.create_timer(args.report_period, self.print_report)
        self.param_timer = self.create_timer(2.0, self.refresh_params)

        if args.duration > 0.0:
            self.stop_timer = self.create_timer(args.duration, self.stop_and_report)
        else:
            self.stop_timer = None

    def _setup_param_client(self) -> None:
        node_name = self.args.slam_node.rstrip("/")
        if not node_name.startswith("/"):
            node_name = f"/{node_name}"
        self._param_service = f"{node_name}/get_parameters"
        self._param_client = self.create_client(GetParameters, self._param_service)

    def tf_callback(self, msg: TFMessage) -> None:
        self.tf_message_count += 1
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, "diagnose_slam_scan_gates")

    def tf_static_callback(self, msg: TFMessage) -> None:
        self.tf_static_message_count += 1
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, "diagnose_slam_scan_gates")

    def refresh_params(self) -> None:
        if self._param_client is None or self._param_future is not None:
            return
        if not self._param_client.wait_for_service(timeout_sec=0.0):
            return

        request = GetParameters.Request()
        request.names = [
            "odom_frame",
            "base_frame",
            "throttle_scans",
            "minimum_time_interval",
            "minimum_travel_distance",
            "minimum_travel_heading",
            "min_laser_range",
            "max_laser_range",
            "map_update_interval",
            "paused_new_measurements",
        ]
        self._param_future = self._param_client.call_async(request)
        self._param_future.add_done_callback(self._handle_param_response)

    def _handle_param_response(self, future) -> None:
        self._param_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"parameter query failed: {exc}")
            return

        request_names = [
            "odom_frame",
            "base_frame",
            "throttle_scans",
            "minimum_time_interval",
            "minimum_travel_distance",
            "minimum_travel_heading",
            "min_laser_range",
            "max_laser_range",
            "map_update_interval",
            "paused_new_measurements",
        ]
        for name, value in zip(request_names, response.values):
            parsed = self._parse_parameter_value(value)
            if parsed is not None:
                self.cfg[name] = parsed
        self.cfg_source = self._param_service

    @staticmethod
    def _parse_parameter_value(value):
        if value.type == ParameterType.PARAMETER_STRING:
            return value.string_value
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return value.integer_value
        if value.type == ParameterType.PARAMETER_BOOL:
            return value.bool_value
        return None

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_publish_count += 1
        self.last_map_stamp = Time.from_msg(msg.header.stamp)
        payload = bytearray()
        for value in msg.data:
            payload.append((value + 1) & 0xFF)
        digest = zlib.adler32(payload)
        if digest != self.last_map_hash:
            self.map_change_count += 1
            self.last_map_hash = digest
            self.last_map_change_stamp = self.last_map_stamp

    def scan_callback(self, scan: LaserScan) -> None:
        self.total_scans += 1
        stamp = Time.from_msg(scan.header.stamp)

        layer1 = self._evaluate_layer1(scan, stamp)
        if layer1 is None:
            return
        base_pose, sensor_pose = layer1

        if not self._evaluate_layer2(stamp, base_pose):
            return

        if not self._evaluate_layer3(scan, stamp, sensor_pose):
            return

        self._evaluate_layer4(scan)

    def _evaluate_layer1(
        self, scan: LaserScan, stamp: Time
    ) -> Optional[Tuple[Pose2D, Pose2D]]:
        if not scan.header.frame_id:
            self.layer1_reject["empty_scan_frame"] += 1
            return None
        self.scan_frame_counter[scan.header.frame_id] += 1

        ok, debug = self.tf_buffer.can_transform(
            self.cfg["odom_frame"],
            scan.header.frame_id,
            stamp,
            timeout=Duration(seconds=0.0),
            return_debug_tuple=True,
        )
        if not ok:
            reason = self._classify_tf_debug(debug)
            self.layer1_reject[f"scan_to_odom_tf:{reason}"] += 1
            self.layer1_debug_counter[f"scan_to_odom_tf:{debug}"] += 1
            self._record_layer1_sample(
                "scan_to_odom_tf",
                scan.header.frame_id,
                stamp,
                debug,
            )
            return None

        ok, debug = self.tf_buffer.can_transform(
            self.cfg["odom_frame"],
            self.cfg["base_frame"],
            stamp,
            timeout=Duration(seconds=0.0),
            return_debug_tuple=True,
        )
        if not ok:
            reason = self._classify_tf_debug(debug)
            self.layer1_reject[f"base_to_odom_tf:{reason}"] += 1
            self.layer1_debug_counter[f"base_to_odom_tf:{debug}"] += 1
            self._record_layer1_sample(
                "base_to_odom_tf",
                self.cfg["base_frame"],
                stamp,
                debug,
            )
            return None

        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.cfg["odom_frame"],
                self.cfg["base_frame"],
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.layer1_reject[f"base_to_odom_lookup:{type(exc).__name__}"] += 1
            self.layer1_debug_counter[f"base_to_odom_lookup:{exc}"] += 1
            self._record_layer1_sample(
                "base_to_odom_lookup",
                self.cfg["base_frame"],
                stamp,
                str(exc),
            )
            return None

        try:
            sensor_tf = self.tf_buffer.lookup_transform(
                self.cfg["odom_frame"],
                scan.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.layer1_reject[f"sensor_to_odom_tf:{type(exc).__name__}"] += 1
            return None

        try:
            self.tf_buffer.lookup_transform(
                self.cfg["base_frame"],
                scan.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.layer1_reject[f"scan_to_base_tf:{type(exc).__name__}"] += 1
            self.layer1_debug_counter[f"scan_to_base_tf:{exc}"] += 1
            self._record_layer1_sample(
                "scan_to_base_tf",
                scan.header.frame_id,
                stamp,
                str(exc),
            )
            return None

        return self._pose_from_transform(base_tf), self._pose_from_transform(sensor_tf)

    def _evaluate_layer2(self, stamp: Time, base_pose: Pose2D) -> bool:
        self.outer_scan_ctr += 1

        if self.outer_first_measurement:
            self.outer_last_stamp = stamp
            self.outer_last_pose = base_pose
            self.outer_first_measurement = False
            self.layer2_accept += 1
            return True

        if bool(self.cfg["paused_new_measurements"]):
            self.layer2_reject["paused_new_measurements"] += 1
            return False

        throttle = max(1, int(self.cfg["throttle_scans"]))
        if (self.outer_scan_ctr % throttle) != 0:
            self.layer2_reject["throttle_scans"] += 1
            return False

        minimum_time = float(self.cfg["minimum_time_interval"])
        if self.outer_last_stamp is not None:
            dt = (stamp - self.outer_last_stamp).nanoseconds / 1e9
            if dt < minimum_time:
                self.layer2_reject["minimum_time_interval"] += 1
                return False

        minimum_travel_distance = float(self.cfg["minimum_travel_distance"])
        min_dist2 = minimum_travel_distance * minimum_travel_distance
        dist2 = 0.0
        if self.outer_last_pose is not None:
            dist2 = self.outer_last_pose.squared_distance(base_pose)

        if self.outer_scan_ctr < 5:
            self.layer2_reject["warmup_first_4_scans"] += 1
            return False

        if dist2 < 0.8 * min_dist2:
            self.layer2_reject["minimum_travel_distance_prefilter"] += 1
            return False

        self.outer_last_pose = base_pose
        self.outer_last_stamp = stamp
        self.layer2_accept += 1
        return True

    def _evaluate_layer3(self, scan: LaserScan, stamp: Time, sensor_pose: Pose2D) -> bool:
        validate_reason = self._validate_karto_scan(scan)
        if validate_reason is not None:
            self.layer3_reject[validate_reason] += 1
            return False

        if self.karto_first_scan:
            self.karto_first_scan = False
            self.karto_last_stamp = stamp
            self.karto_last_sensor_pose = sensor_pose
            self.layer3_accept += 1
            return True

        if self.karto_last_stamp is None or self.karto_last_sensor_pose is None:
            self.karto_last_stamp = stamp
            self.karto_last_sensor_pose = sensor_pose
            self.layer3_accept += 1
            return True

        dt = (stamp - self.karto_last_stamp).nanoseconds / 1e9
        if dt >= KARTO_MINIMUM_TIME_INTERVAL_ASSUMED:
            self.karto_last_stamp = stamp
            self.karto_last_sensor_pose = sensor_pose
            self.layer3_accept += 1
            return True

        delta_heading = abs(normalize_angle(sensor_pose.yaw - self.karto_last_sensor_pose.yaw))
        if delta_heading >= float(self.cfg["minimum_travel_heading"]):
            self.karto_last_stamp = stamp
            self.karto_last_sensor_pose = sensor_pose
            self.layer3_accept += 1
            return True

        dist2 = self.karto_last_sensor_pose.squared_distance(sensor_pose)
        min_dist = float(self.cfg["minimum_travel_distance"])
        if dist2 >= (min_dist * min_dist):
            self.karto_last_stamp = stamp
            self.karto_last_sensor_pose = sensor_pose
            self.layer3_accept += 1
            return True

        self.layer3_reject["karto_has_moved_enough"] += 1
        return False

    def _validate_karto_scan(self, scan: LaserScan) -> Optional[str]:
        if not scan.ranges:
            return "karto_validate_empty_ranges"
        if not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0.0:
            return "karto_validate_bad_angle_increment"
        if not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_max):
            return "karto_validate_bad_angle_bounds"

        angular_range = abs(scan.angle_max - scan.angle_min)
        eps = sys.float_info.epsilon
        is_360 = abs(angular_range - 2.0 * math.pi) < (
            scan.angle_increment - (eps * 2.0 * math.pi)
        )
        if angular_range > 6.10865 and round(angular_range / scan.angle_increment) + 1 == len(scan.ranges):
            is_360 = False

        residual = 0 if is_360 else 1
        expected = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment) + residual)
        if expected != len(scan.ranges):
            return "karto_validate_range_count_mismatch"

        min_laser_range = max(0.0, float(self.cfg["min_laser_range"]))
        min_laser_range = max(min_laser_range, float(scan.range_min))
        max_laser_range = float(self.cfg["max_laser_range"])
        if max_laser_range <= 0.0 or max_laser_range > float(scan.range_max):
            max_laser_range = float(scan.range_max)
        if not (min_laser_range <= max_laser_range <= float(scan.range_max)):
            return "karto_validate_bad_range_threshold"

        return None

    def _evaluate_layer4(self, scan: LaserScan) -> None:
        min_laser_range = max(0.0, float(self.cfg["min_laser_range"]))
        min_laser_range = max(min_laser_range, float(scan.range_min))

        max_laser_range = float(self.cfg["max_laser_range"])
        if max_laser_range <= 0.0 or max_laser_range > float(scan.range_max):
            max_laser_range = float(scan.range_max)
        range_threshold = max_laser_range
        max_range = float(scan.range_max)

        usable = 0
        endpoint_hits = 0
        clipped = 0
        ignored = 0
        for reading in scan.ranges:
            self.beam_total += 1
            if math.isnan(reading) or reading <= min_laser_range or reading >= max_range:
                ignored += 1
                self.beam_ignored += 1
                continue
            usable += 1
            if reading >= range_threshold:
                clipped += 1
                self.beam_clipped += 1
            else:
                endpoint_hits += 1
                self.beam_endpoint_hit += 1

        if usable == 0:
            self.accepted_scans_zero_usable_beams += 1
        if endpoint_hits == 0:
            self.accepted_scans_zero_endpoint_hits += 1

    def print_report(self) -> None:
        total = max(1, self.total_scans)
        self.get_logger().info("")
        self.get_logger().info("========== slam scan gate report ==========")
        self.get_logger().info(
            "config source=%s scan_topic=%s map_topic=%s slam_node=%s tf_topic=%s tf_static_topic=%s"
            % (
                self.cfg_source,
                self.args.scan_topic,
                self.args.map_topic,
                self.args.slam_node,
                self.tf_topic,
                self.tf_static_topic,
            )
        )
        self.get_logger().info(
            "tf messages seen=%d tf_static messages seen=%d"
            % (self.tf_message_count, self.tf_static_message_count)
        )
        self.get_logger().info(
            "odom_frame=%s base_frame=%s throttle=%s min_time=%.3f min_dist=%.3f min_heading=%.3f"
            % (
                self.cfg["odom_frame"],
                self.cfg["base_frame"],
                self.cfg["throttle_scans"],
                float(self.cfg["minimum_time_interval"]),
                float(self.cfg["minimum_travel_distance"]),
                float(self.cfg["minimum_travel_heading"]),
            )
        )
        self.get_logger().info(
            "karto_minimum_time_interval is assumed %.1fs for slam_toolbox 2.6.10 source parity"
            % KARTO_MINIMUM_TIME_INTERVAL_ASSUMED
        )
        self.get_logger().info("scans seen=%d" % self.total_scans)
        if self.scan_frame_counter:
            for frame_id, count in self.scan_frame_counter.most_common():
                self.get_logger().info(
                    "scan frame_id %s: %d (%.1f%% of seen)"
                    % (frame_id, count, 100.0 * count / total)
                )
        self._print_counter("layer1 tf/filter reject", self.layer1_reject, total)
        for key, value in self.layer1_debug_counter.most_common(5):
            self.get_logger().info("  layer1 raw debug: %s [%d]" % (key, value))
        if self.layer1_samples:
            self.get_logger().info("  layer1 recent samples:")
            for sample in self.layer1_samples:
                self.get_logger().info("    %s" % sample)
        tf_frames = self._available_tf_frames()
        if tf_frames:
            self.get_logger().info("  tf buffer frames: %s" % ", ".join(tf_frames[:20]))
        self._print_counter("layer2 outer prefilter reject", self.layer2_reject, total)
        self.get_logger().info(
            "layer2 accepted=%d (%.1f%% of seen)"
            % (self.layer2_accept, 100.0 * self.layer2_accept / total)
        )
        self._print_counter("layer3 karto reject", self.layer3_reject, total)
        self.get_logger().info(
            "layer3 accepted=%d (%.1f%% of seen)"
            % (self.layer3_accept, 100.0 * self.layer3_accept / total)
        )

        accepted = max(1, self.layer3_accept)
        self.get_logger().info(
            "layer4 beam stats on accepted scans: total=%d ignored=%d clipped_no_hit=%d endpoint_hits=%d"
            % (
                self.beam_total,
                self.beam_ignored,
                self.beam_clipped,
                self.beam_endpoint_hit,
            )
        )
        if self.beam_total > 0:
            self.get_logger().info(
                "layer4 beam ratios: ignored=%.1f%% clipped_no_hit=%.1f%% endpoint_hit=%.1f%%"
                % (
                    100.0 * self.beam_ignored / self.beam_total,
                    100.0 * self.beam_clipped / self.beam_total,
                    100.0 * self.beam_endpoint_hit / self.beam_total,
                )
            )
        self.get_logger().info(
            "accepted scans with zero usable beams=%d (%.1f%% of layer3 accepts)"
            % (
                self.accepted_scans_zero_usable_beams,
                100.0 * self.accepted_scans_zero_usable_beams / accepted,
            )
        )
        self.get_logger().info(
            "accepted scans with zero endpoint hits=%d (%.1f%% of layer3 accepts)"
            % (
                self.accepted_scans_zero_endpoint_hits,
                100.0 * self.accepted_scans_zero_endpoint_hits / accepted,
            )
        )

        identical_publishes = self.map_publish_count - self.map_change_count
        self.get_logger().info(
            "map publishes=%d content_changes=%d identical_publishes=%d"
            % (self.map_publish_count, self.map_change_count, identical_publishes)
        )
        if self.last_map_change_stamp is not None:
            age = (self.get_clock().now() - self.last_map_change_stamp).nanoseconds / 1e9
            self.get_logger().info("last map content change %.2fs ago" % age)

        bottleneck = self._dominant_bottleneck()
        self.get_logger().info("dominant bottleneck guess: %s" % bottleneck)
        self.get_logger().info("===========================================")

    def _dominant_bottleneck(self) -> str:
        candidates = []
        if self.layer1_reject:
            candidates.append(("layer1_tf_filter", sum(self.layer1_reject.values())))
        if self.layer2_reject:
            candidates.append(("layer2_outer_prefilter", sum(self.layer2_reject.values())))
        if self.layer3_reject:
            candidates.append(("layer3_karto_gate", sum(self.layer3_reject.values())))
        if self.layer3_accept > 0:
            candidates.append(("layer4_zero_endpoint_hits", self.accepted_scans_zero_endpoint_hits))
        if not candidates:
            return "no obvious bottleneck yet"
        candidates.sort(key=lambda item: item[1], reverse=True)
        return candidates[0][0]

    def _print_counter(self, title: str, counter: Counter, total: int) -> None:
        total_count = sum(counter.values())
        self.get_logger().info(
            "%s=%d (%.1f%% of seen)"
            % (title, total_count, 100.0 * total_count / total)
        )
        for key, value in counter.most_common():
            self.get_logger().info(
                "  %s: %d (%.1f%% of seen)"
                % (key, value, 100.0 * value / total)
            )

    def _record_layer1_sample(
        self,
        stage: str,
        frame_id: str,
        stamp: Time,
        debug: str,
    ) -> None:
        latest_ok, latest_debug = self.tf_buffer.can_transform(
            self.cfg["odom_frame"],
            frame_id,
            Time(),
            timeout=Duration(seconds=0.0),
            return_debug_tuple=True,
        )
        latest_status = "ok_at_latest" if latest_ok else latest_debug
        self.layer1_samples.append(
            "%s frame=%s stamp=%.3f reason=%s latest=%s"
            % (
                stage,
                frame_id,
                stamp.nanoseconds / 1e9,
                debug,
                latest_status,
            )
        )

    @staticmethod
    def _classify_tf_debug(debug: str) -> str:
        text = str(debug)
        if "earlier than all the data in the transform cache" in text:
            return "earlier_than_tf_cache"
        if "into the future" in text:
            return "extrapolation_future"
        if "into the past" in text:
            return "extrapolation_past"
        if "does not exist" in text:
            return "frame_missing"
        if "No transform" in text or "No connection" in text:
            return "no_connection"
        if "Invalid frame ID" in text:
            return "invalid_frame_id"
        return "other_tf_debug"

    def _available_tf_frames(self):
        try:
            yaml_text = self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return []
        frames = []
        for line in yaml_text.splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if line.startswith(" ") or line.startswith("\t"):
                continue
            if stripped.endswith(":"):
                frames.append(stripped[:-1])
        return frames

    @staticmethod
    def _pose_from_transform(transform) -> Pose2D:
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return Pose2D(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            yaw=yaw,
        )

    def stop_and_report(self) -> None:
        self.print_report()
        self.get_logger().info("duration reached, shutting down")
        self.destroy_node()
        rclpy.shutdown()


def parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Estimate where obstacle_scan messages are gated before becoming map updates."
    )
    parser.add_argument("--scan-topic", default="obstacle_scan")
    parser.add_argument("--map-topic", default="map")
    parser.add_argument("--slam-node", default="/slam_toolbox")
    parser.add_argument("--tf-topic", default=None)
    parser.add_argument("--tf-static-topic", default=None)
    parser.add_argument("--report-period", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.tf_topic is None:
        args.tf_topic = derive_tf_topic(args.slam_node, "tf")
    if args.tf_static_topic is None:
        args.tf_static_topic = derive_tf_topic(args.slam_node, "tf_static")
    return args


def derive_tf_topic(slam_node: str, leaf: str) -> str:
    cleaned = slam_node.strip()
    if not cleaned:
        return f"/{leaf}"
    if not cleaned.startswith("/"):
        cleaned = f"/{cleaned}"
    ns = cleaned.rsplit("/", 1)[0]
    if not ns:
        return f"/{leaf}"
    return f"{ns}/{leaf}"


def main(argv=None) -> int:
    argv = argv if argv is not None else sys.argv
    args = parse_args(rclpy.utilities.remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv)
    node = SlamScanGateDiagnostics(args)
    node.refresh_params()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_report()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
