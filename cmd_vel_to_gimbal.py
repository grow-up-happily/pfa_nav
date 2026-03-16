#!/usr/bin/env python3
"""订阅 /cmd_vel 话题，将 vx/vy 通过 VisionToGimbal 协议发送到 /dev/gimbal 串口。"""

import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CmdVelToGimbal(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_gimbal')

        self.declare_parameter('serial_port', '/dev/gimbal')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('send_rate', 50.0)
        self.declare_parameter('timeout', 0.5)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baud_rate').value)
        topic = self.get_parameter('cmd_vel_topic').value
        send_rate = float(self.get_parameter('send_rate').value)
        self.timeout = float(self.get_parameter('timeout').value)

        try:
            self.ser = serial.Serial(port, baud)
            self.get_logger().info(f'串口已打开: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            self.ser = None

        self.vx = 0.0
        self.vy = 0.0
        self.last_cmd_time = 0.0

        self.create_subscription(Twist, topic, self.cmd_vel_callback, 10)
        self.create_timer(1.0 / send_rate, self.send_loop)
        self.get_logger().info(f'已订阅 {topic}')

    def _crc16(self, data: bytes) -> int:
        """CRC-CCITT (0x8408 反射多项式)，与 C++ tools::get_crc16 一致。"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0x8408
                else:
                    crc >>= 1
        return crc & 0xFFFF

    def cmd_vel_callback(self, msg: Twist):
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.last_cmd_time = time.monotonic()

    def _build_packet(self, vx: float, vy: float) -> bytes:
        """按 VisionToGimbal 协议打包当前底盘速度。"""
        # VisionToGimbal 协议: head(2B) + mode(1B) + 8f(32B) + crc16(2B) = 37B
        # gimbal.send(false, false, 0, 0, 0, 0, 0, 0, vx, vy)
        # control=false → mode=0
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

    def send_loop(self):
        if self.ser is None:
            return

        now = time.monotonic()
        if now - self.last_cmd_time > self.timeout:
            vx = 0.0
            vy = 0.0
        else:
            vx = self.vx
            vy = self.vy

        packet = self._build_packet(vx, vy)

        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f'串口发送失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToGimbal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
