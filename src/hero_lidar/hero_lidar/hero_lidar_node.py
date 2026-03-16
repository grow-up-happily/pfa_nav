#!/usr/bin/env python3
import math
import os
import struct
from collections import deque
from statistics import median
import yaml

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
import serial
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class WaypointStaticPublisher(Node):
    """持续发布 map->base TF，支持 waypoint 初始值和 hero_pose 更新。"""

    def __init__(self):
        super().__init__('waypoint_static_publisher')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('pose_topic', '/hero_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('distance_median_window', 7)
        self.declare_parameter('distance_median_deadband', 0.05)

        self.parent_frame = 'map'
        self.child_frame = 'base'

        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_pose = None
        self.serial_device = self.get_parameter('serial_port').value
        self.serial_port = None
        self.distance_median_window = int(
            self.get_parameter('distance_median_window').value
        )
        self.distance_median_deadband = float(
            self.get_parameter('distance_median_deadband').value
        )
        if self.distance_median_window < 1:
            self.get_logger().warning(
                'distance_median_window must be >= 1, fallback to 1.'
            )
            self.distance_median_window = 1
        if self.distance_median_window % 2 == 0:
            self.distance_median_window += 1
            self.get_logger().warning(
                'distance_median_window should be odd, auto-adjusted to '
                f'{self.distance_median_window}.'
            )
        if self.distance_median_deadband < 0.0:
            self.get_logger().warning(
                'distance_median_deadband must be >= 0, fallback to 0.0.'
            )
            self.distance_median_deadband = 0.0
        self.distance_history = deque(maxlen=self.distance_median_window)

        self._init_serial()

        waypoints_file = self.get_parameter('waypoints_file').value
        self._load_initial_pose_from_waypoints(waypoints_file)

        pose_topic = self.get_parameter('pose_topic').value
        if pose_topic:
            self.pose_sub = self.create_subscription(
                PoseStamped,
                pose_topic,
                self.goal_pose_callback,
                10,
            )
            self.get_logger().info(
                f'Listening for hero_pose PoseStamped updates on {pose_topic}.'
            )

        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            self.get_logger().warning(
                f'Invalid publish_rate={publish_rate}, fallback to 10.0 Hz.'
            )
            publish_rate = 10.0

        self.create_timer(1.0 / publish_rate, self.publish_transform)
        self.create_timer(1.0, self._print_chassis_base)
        self.create_timer(2.0, self._check_serial_connection)

    def _check_serial_connection(self):
        if self.serial_port is None:
            self.get_logger().info(f'Trying to connect to serial port {self.serial_device}...')
            self._init_serial()

    def _init_serial(self):
        if self.serial_port is not None:
           return

        baud_rate = int(self.get_parameter('serial_baud_rate').value)
        try:
            self.serial_port = serial.Serial(self.serial_device, baud_rate, timeout=0.1, write_timeout=0.1)
            self.get_logger().info(
                f'Opened gimbal serial port {self.serial_device} @ {baud_rate}.'
            )
        except Exception as exc:
            self.serial_port = None
            self.get_logger().warn(
                f'Failed to open gimbal serial port {self.serial_device}: {exc}'
            )

    def _crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def _send_gimbal_packet(self, vx: float, vy: float):
        if self.serial_port is None:
            return

        payload = struct.pack(
            '<2sBffffffff',
            b'SP',
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(vx),
            float(vy),
        )
        packet = payload + struct.pack('<H', self._crc16(payload))

        try:
            self.serial_port.write(packet)
        except Exception:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    def _filter_distance_against_history_median(self, x: float, y: float):
        raw_distance = math.hypot(x, y)
        raw_heading = math.atan2(y, x)

        if self.distance_history:
            history_median = median(self.distance_history)
            if abs(raw_distance - history_median) <= self.distance_median_deadband:
                filtered_distance = history_median
            else:
                filtered_distance = raw_distance
        else:
            history_median = raw_distance
            filtered_distance = raw_distance

        self.distance_history.append(raw_distance)
        filtered_x = filtered_distance * math.cos(raw_heading)
        filtered_y = filtered_distance * math.sin(raw_heading)
        return raw_distance, history_median, filtered_distance, filtered_x, filtered_y

    def _load_initial_pose_from_waypoints(self, waypoints_file: str):
        waypoint_path = waypoints_file or os.path.join(os.getcwd(), 'waypoints.yaml')
        if not os.path.exists(waypoint_path):
            if waypoints_file:
                self.get_logger().error(f'waypoints file not found: {waypoint_path}')
            else:
                self.get_logger().warning(
                    f'Waypoint file not found: {waypoint_path}. Waiting for hero_pose updates.'
                )
            return

        try:
            with open(waypoint_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
        except Exception as exc:
            self.get_logger().error(f'failed to parse yaml: {exc}')
            return

        waypoint = self._extract_first_waypoint(data)
        if waypoint is None:
            self.get_logger().error(f'no waypoint entries found in {waypoint_path}')
            return

        raw_x = self._ci_key_lookup(waypoint, {'posx', 'x', 'px'})
        raw_y = self._ci_key_lookup(waypoint, {'posy', 'y', 'py'})
        raw_z = self._ci_key_lookup(waypoint, {'posz', 'z', 'pz'})
        raw_qx = self._ci_key_lookup(waypoint, {'orix', 'qx'})
        raw_qy = self._ci_key_lookup(waypoint, {'oriy', 'qy'})
        raw_qz = self._ci_key_lookup(waypoint, {'oriz', 'qz'})
        raw_qw = self._ci_key_lookup(waypoint, {'oriw', 'qw'})

        try:
            self._update_pose(
                raw_x,
                raw_y,
                raw_z if raw_z is not None else 0.0,
                raw_qx if raw_qx is not None else 0.0,
                raw_qy if raw_qy is not None else 0.0,
                raw_qz if raw_qz is not None else 0.0,
                raw_qw if raw_qw is not None else 1.0,
                f'waypoint file {waypoint_path}',
            )
        except Exception as exc:
            self.get_logger().error(f'failed to parse first waypoint coordinates: {exc}')
            return

    def _extract_first_waypoint(self, data):
        first = None

        if isinstance(data, dict):
            for key in sorted(data.keys()):
                if isinstance(key, str) and key.lower().startswith('waypoint') and isinstance(data[key], dict):
                    first = data[key]
                    break

            if first is None:
                for value in data.values():
                    if isinstance(value, dict):
                        first = value
                        break

        elif isinstance(data, (list, tuple)) and len(data) > 0:
            first = data[0]

        if isinstance(first, dict):
            return first
        return None

    def _ci_key_lookup(self, data, targets):
        for key, value in data.items():
            normalized_key = ''.join(key.lower().split('_'))
            if normalized_key in targets:
                return value
        return None

    def _update_pose(self, x, y, z, qx, qy, qz, qw, source: str):
        self.base_pose = {
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'qx': float(qx),
            'qy': float(qy),
            'qz': float(qz),
            'qw': float(qw),
        }
        self.get_logger().info(
            f'Updated {self.parent_frame}->{self.child_frame} from {source}: '
            f'x={self.base_pose["x"]:.3f}, y={self.base_pose["y"]:.3f}'
        )

    def goal_pose_callback(self, msg: PoseStamped):
        frame_id = msg.header.frame_id or 'map'
        if frame_id not in ('map', self.parent_frame):
            self.get_logger().warning(
                f'Ignoring PoseStamped in frame {frame_id}, expected map or {self.parent_frame}.'
            )
            return

        pose = msg.pose
        self._update_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            'hero_pose',
        )

    def publish_transform(self):
        if self.base_pose is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.base_pose['x']
        transform.transform.translation.y = self.base_pose['y']
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = self.base_pose['qx']
        transform.transform.rotation.y = self.base_pose['qy']
        transform.transform.rotation.z = self.base_pose['qz']
        transform.transform.rotation.w = self.base_pose['qw']

        self.broadcaster.sendTransform(transform)

    def _print_chassis_base(self):
        serial_status = (
            f'Serial: {self.serial_device} connected'
            if self.serial_port is not None
            else ''
        )

        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time(), timeout=Duration(seconds=0.0))
            p = t.transform.translation
            d = (p.x**2 + p.y**2)**0.5
            msg_mo = f'map->odom:({p.x:.2f},{p.y:.2f},d={d:.2f})'
        except Exception:
            msg_mo = 'map->odom:N/A'

        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.0))
            p = t.transform.translation
            d = (p.x**2 + p.y**2)**0.5
            msg_ob = f'odom->base_footprint:({p.x:.2f},{p.y:.2f},d={d:.2f})'
        except Exception:
            msg_ob = 'odom->base_footprint:N/A'

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=0.0))
            p = t.transform.translation
            d = (p.x**2 + p.y**2)**0.5
            msg_mb = f'map->base_footprint:({p.x:.2f},{p.y:.2f},d={d:.2f})'
        except Exception:
            msg_mb = 'map->base_footprint:N/A'

        target_frame = 'base_footprint'
        source_frame = 'base'
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            translation = trans.transform.translation
            translation.z = 0.0
            raw_distance, history_median, filtered_distance, send_x, send_y = (
                self._filter_distance_against_history_median(translation.x, translation.y)
            )
            self.get_logger().info(
                f'{serial_status} | {msg_mo}\n'
                f'{msg_ob}\n'
                f'{msg_mb}\n'
                f'{target_frame}->{source_frame}: '
                f'dist_raw={raw_distance:.3f} | '
                f'dist_med={history_median:.3f} | '
                f'dist_send={filtered_distance:.3f} | '
                f'trans=({translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}) | '
                f'send: vx={send_x:.3f}, vy={send_y:.3f}, vz={translation.z:.3f}'
            )
            self._send_gimbal_packet(send_x, send_y)
        except Exception as exc:
            pass

    def save_pose_to_waypoints(self):
        if self.base_pose is None:
            return

        waypoints_file = self.get_parameter('waypoints_file').value
        waypoint_path = waypoints_file or os.path.join(os.getcwd(), 'waypoints.yaml')

        data = {
            'waypoint_1': {
                'x': self.base_pose['x'],
                'y': self.base_pose['y'],
                'z': self.base_pose['z'],
                'qx': self.base_pose['qx'],
                'qy': self.base_pose['qy'],
                'qz': self.base_pose['qz'],
                'qw': self.base_pose['qw'],
            }
        }

        try:
            with open(waypoint_path, 'w', encoding='utf-8') as file:
                yaml.dump(data, file, default_flow_style=False)
            self.get_logger().info(f'Saved current pose to {waypoint_path}')
        except Exception as exc:
            self.get_logger().error(f'Failed to save pose to {waypoint_path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointStaticPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.save_pose_to_waypoints()
            if node.serial_port is not None:
                node.serial_port.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
