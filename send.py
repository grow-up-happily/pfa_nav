#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading

class VelocitySender(Node):
    def __init__(self):
        super().__init__('velocity_sender')
        
        # 串口配置
        self.serial_port = "/dev/ttyUSB0"  # 替换为你的串口路径
        self.baud_rate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"串口连接成功: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            self.ser = None
        
        # 订阅速度指令话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.smooth_cmd_vel_sub = self.create_subscription(
            Twist, '/smooth_cmd_vel', self.cmd_vel_callback, 10)
        
        # 启动串口接收线程
        self.receiver_thread = threading.Thread(target=self.serial_receiver)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        self.get_logger().info("速度发送节点已启动")
        self.get_logger().info("已订阅 /cmd_vel 话题")
        self.get_logger().info("已订阅 /smooth_cmd_vel 话题")

    def cmd_vel_callback(self, msg):
        """处理速度指令并发送到串口"""
        if self.ser is None:
            return
            
        # 获取x, y线速度
        x_vel = int(msg.linear.x * 1000)
        y_vel = int(msg.linear.y * 1000)
        
        # 封装成数据包
        x_sign = '+' if x_vel >= 0 else '-'
        y_sign = '+' if y_vel >= 0 else '-'
        
        x_vel = abs(x_vel)
        y_vel = abs(y_vel)
        
        data_packet = f"S{x_sign}{x_vel:03d}{y_sign}{y_vel:03d}{0}E"
        
        # 打印调试信息
        self.get_logger().info(f"收到平滑速度指令 - vx: {msg.linear.x:.3f}, vy: {msg.linear.y:.3f}")
        self.get_logger().info(f"发送数据包: {data_packet}")
        
        # 发送数据包
        try:
            self.ser.write(data_packet.encode())
        except Exception as e:
            self.get_logger().error(f"串口发送失败: {e}")

    def serial_receiver(self):
        """持续监听串口数据并写入文件"""
        while rclpy.ok() and self.ser:
            try:
                # 读取所有可用数据
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                    self.get_logger().info(f"收到串口数据: {data}")
                    
                    # 实时写入并覆盖文件
                    with open('data.txt', 'w') as f:
                        f.write(data)
                        
            except Exception as e:
                self.get_logger().error(f"串口接收错误: {str(e)}")
                break

def main(args=None):
    rclpy.init(args=args)
    sender = VelocitySender()
    
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        sender.get_logger().info("速度发送节点已停止")
    finally:
        # 确保程序退出时关闭串口
        if hasattr(sender, 'ser') and sender.ser:
            sender.ser.close()
        sender.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
