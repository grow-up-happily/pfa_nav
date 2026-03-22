#!/usr/bin/env python3
"""
整合 hp_nav + cmd_vel_to_gimbal：
  - 串口接收：持续读取血量数据，根据血量切换导航点
  - 串口发送：订阅 /cmd_vel，按 VisionToGimbal 协议发送 vx/vy 到云台
  - 共用同一个串口，线程安全
"""

import struct
import time
import threading
import argparse

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import serial
import yaml


class HpGimbalNavNode(Node):
    def __init__(self, serial_port, baud_rate, yaml_path,
                 point_normal, point_low_hp, hp_threshold,
                 cmd_vel_topic, send_rate, cmd_timeout):
        super().__init__('hp_gimbal_nav_node')

        # ── 加载航点 ──
        self.waypoints = self.load_waypoints(yaml_path)
        self.get_logger().info(f"已从 {yaml_path} 加载 {len(self.waypoints)} 个航点")

        self.point_normal = point_normal - 1
        self.point_low_hp = point_low_hp - 1
        for idx, label in [(self.point_normal, '正常'), (self.point_low_hp, '低血量')]:
            if idx < 0 or idx >= len(self.waypoints):
                raise ValueError(f"{label}点编号超出范围 [1-{len(self.waypoints)}]")

        self.hp_threshold = hp_threshold
        self.current_hp = 600
        self.current_target = self.point_normal

        self.get_logger().info(
            f"正常点: {self.waypoints[self.point_normal].get('Name', self.point_normal+1)}, "
            f"低血量点: {self.waypoints[self.point_low_hp].get('Name', self.point_low_hp+1)}, "
            f"血量阈值: {self.hp_threshold}"
        )

        # ── 串口（共用） ──
        self.ser = None
        self.serial_lock = threading.Lock()
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"串口连接成功: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")

        # ── 导航客户端 ──
        self.nav_ac = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')
        self.sending = False
        self.max_retry = 3
        self.retry_count = 0

        # ── cmd_vel 发送相关 ──
        self.vx = 0.0
        self.vy = 0.0
        self.last_cmd_time = 0.0
        self.cmd_timeout = cmd_timeout

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_timer(1.0 / send_rate, self.send_cmd_loop)
        self.get_logger().info(f"已订阅 {cmd_vel_topic}，发送频率 {send_rate} Hz")

        # ── 串口接收线程 ──
        if self.ser:
            self.receiver_thread = threading.Thread(target=self.serial_receiver, daemon=True)
            self.receiver_thread.start()

        # ── 导航循环定时器 ──
        self.timer = self.create_timer(1.0, self.navigation_loop)
        self.get_logger().info("hp_gimbal_nav 节点已启动")

    # ─────────────── 航点加载 ───────────────

    def load_waypoints(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        wps = []
        for i in range(1, data['Waypoints_Num'] + 1):
            wps.append(data[f'Waypoint_{i}'])
        return wps

    # ─────────────── 串口接收（血量） ───────────────

    def serial_receiver(self):
        buf = b''
        while rclpy.ok() and self.ser:
            try:
                with self.serial_lock:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        buf += self.ser.read(waiting)
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        hp = int(line)
                        self.current_hp = hp
                        self.get_logger().info(f"血量更新: {hp}")
                    except ValueError:
                        self.get_logger().warn(f"无法解析血量数据: {line}")
            except serial.SerialException as e:
                self.get_logger().error(f"串口通信错误: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"串口接收错误: {e}")
                break

    # ─────────────── 串口发送（cmd_vel → 云台） ───────────────

    def cmd_vel_callback(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.last_cmd_time = time.monotonic()

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

    def _build_packet(self, vx: float, vy: float) -> bytes:
        payload = struct.pack(
            '<2sB8f',
            b'SP',       # head
            0,           # mode: 0=不控制
            0.0,         # yaw
            0.0,         # yaw_vel
            0.0,         # yaw_acc
            0.0,         # pitch
            0.0,         # pitch_vel
            0.0,         # pitch_acc
            vx,          # vx
            vy,          # vy
        )
        return payload + struct.pack('<H', self._crc16(payload))

    def send_cmd_loop(self):
        if self.ser is None:
            return
        now = time.monotonic()
        if now - self.last_cmd_time > self.cmd_timeout:
            vx, vy = 0.0, 0.0
        else:
            vx, vy = self.vx, self.vy
        packet = self._build_packet(vx, vy)
        try:
            with self.serial_lock:
                self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"串口发送失败: {e}")

    # ─────────────── 导航逻辑 ───────────────

    def navigation_loop(self):
        if self.current_hp < self.hp_threshold:
            desired_target = self.point_low_hp
        else:
            desired_target = self.point_normal

        if desired_target != self.current_target and self.sending:
            self.get_logger().info(
                f"血量={self.current_hp}，切换目标点: "
                f"{self.waypoints[self.current_target].get('Name', '')} -> "
                f"{self.waypoints[desired_target].get('Name', '')}"
            )
            self.sending = False
            self.retry_count = 0

        self.current_target = desired_target

        if not self.sending:
            self.send_goal(self.current_target)

    def send_goal(self, waypoint_idx):
        if self.sending:
            return
        wp = self.waypoints[waypoint_idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['Pos_x'])
        goal.pose.pose.position.y = float(wp['Pos_y'])
        goal.pose.pose.position.z = float(wp['Pos_z'])
        goal.pose.pose.orientation.x = float(wp['Ori_x'])
        goal.pose.pose.orientation.y = float(wp['Ori_y'])
        goal.pose.pose.orientation.z = float(wp['Ori_z'])
        goal.pose.pose.orientation.w = float(wp['Ori_w'])

        self.nav_ac.wait_for_server()
        wp_name = wp.get('Name', f'Waypoint_{waypoint_idx+1}')
        self.get_logger().info(f"发送目标点: {wp_name} (血量={self.current_hp})")

        self._send_goal_future = self.nav_ac.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.sending = True
        self.retry_count = 0

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.retry_count += 1
            if self.retry_count < self.max_retry:
                self.get_logger().warn(f"目标被拒绝，重试第 {self.retry_count} 次...")
                self.sending = False
            else:
                self.get_logger().error(f"目标连续 {self.max_retry} 次被拒绝")
                self.sending = False
                self.retry_count = 0
            return

        self.get_logger().info("目标已被接受，等待到达...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("已到达目标点")
        else:
            self.get_logger().warn(f"导航失败，状态码: {status}")
        self.sending = False
        self.retry_count = 0


def main():
    parser = argparse.ArgumentParser(description='血量导航 + cmd_vel转发云台（共用串口）')
    parser.add_argument('--serial_port', type=str, default='/dev/gimbal', help='串口端口')
    parser.add_argument('--baud_rate', type=int, default=115200, help='串口波特率')
    parser.add_argument('--yaml', type=str, default='waypoints.yaml', help='航点 YAML 文件路径')
    parser.add_argument('--point_normal', type=int, default=1, help='正常血量时的目标点编号(从1开始)')
    parser.add_argument('--point_low_hp', type=int, default=2, help='低血量时的目标点编号(从1开始)')
    parser.add_argument('--hp_threshold', type=int, default=200, help='血量阈值')
    parser.add_argument('--cmd_vel_topic', type=str, default='/cmd_vel', help='cmd_vel 话题')
    parser.add_argument('--send_rate', type=float, default=50.0, help='云台发送频率(Hz)')
    parser.add_argument('--cmd_timeout', type=float, default=0.5, help='cmd_vel 超时时间(秒)')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = HpGimbalNavNode(
            serial_port=args.serial_port,
            baud_rate=args.baud_rate,
            yaml_path=args.yaml,
            point_normal=args.point_normal,
            point_low_hp=args.point_low_hp,
            hp_threshold=args.hp_threshold,
            cmd_vel_topic=args.cmd_vel_topic,
            send_rate=args.send_rate,
            cmd_timeout=args.cmd_timeout,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n节点已停止")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
