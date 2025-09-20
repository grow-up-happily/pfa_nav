#!/usr/bin/env python3

# 仿真跑图
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import serial
import threading
import yaml
import argparse

class GameNode(Node):
    def __init__(self, serial_port, baud_rate, home_idx, order, guard_idx, yaml_path, force_loop=False):
        super().__init__('game_node')
        # 串口配置
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            self.get_logger().info(f"串口连接成功: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")
            self.ser = None
        # 速度指令订阅
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.smooth_cmd_vel_sub = self.create_subscription(
            Twist, '/smooth_cmd_vel', self.cmd_vel_callback, 10)
        # 导航相关
        self.home_idx = home_idx
        self.order = order
        self.guard_idx = guard_idx
        self.waypoints = self.load_wps(yaml_path)
        self.nav_ac = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')
        self.looping = True
        self.loop_idx = 0
        self.sending = False
        self.max_retry = 3 # 最大重试次数
        self.retry_count = 0
        self.status_flag = 0  # 0:未到达, 1:到达
        self.one_shot_mode = False
        self.one_shot_idx = 0
        # 强制循环标志位
        self.force_loop = force_loop
        if self.force_loop:
            self.get_logger().info("强制循环模式已开启，将忽略所有串口指令并执行循环导航")
            # 强制循环模式启动时立即开始循环
            self.looping = True
            self.one_shot_mode = False
        # 启动串口接收线程
        self.receiver_thread = threading.Thread(target=self.serial_receiver)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        self.get_logger().info("速度发送和导航节点已启动")
        self.get_logger().info("已订阅 /cmd_vel 和 /smooth_cmd_vel 话题")
        # 导航循环定时器
        self.timer = self.create_timer(1.0, self.loop_nav)

    def load_wps(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        wps = []
        for i in range(1, data['Waypoints_Num']+1):
            wps.append(data[f'Waypoint_{i}'])
        return wps

    def cmd_vel_callback(self, msg):
        """处理速度指令并发送到串口"""
        if self.ser is None:
            return
        x_vel = int(msg.linear.x * 1000) # 1000倍放大
        y_vel = int(msg.linear.y * 1000) # 1000倍放大
        x_sign = '+' if x_vel >= 0 else '-'
        y_sign = '+' if y_vel >= 0 else '-'
        x_vel = abs(x_vel)
        y_vel = abs(y_vel)
        data_packet = f"S{x_sign}{x_vel:03d}{y_sign}{y_vel:03d}{self.status_flag}E"
        self.get_logger().info(f"收到速度指令 - vx: {msg.linear.x:.3f}, vy: {msg.linear.y:.3f}")
        self.get_logger().info(f"发送数据包: {data_packet}")
        try:
            self.ser.write(data_packet.encode())
        except Exception as e:
            self.get_logger().error(f"串口发送失败: {e}")

    def serial_receiver(self):
        """持续监听串口数据并处理指令"""
        while rclpy.ok() and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('ascii', errors='replace')
                    self.get_logger().info(f"收到串口数据: {data}")
                    with open('data.txt', 'w') as f:
                        f.write(data)
                    for c in data:
                        if self.force_loop:
                            # 强制循环模式：忽略所有串口指令，只执行循环导航
                            self.get_logger().info(f"强制循环模式：忽略指令{c}，继续执行循环导航")
                        else:
                            # 正常模式：根据指令执行不同操作
                            if c == '1':
                                self.get_logger().info("串口收到1，发送home点")
                                rclpy.get_default_context().call_soon_threadsafe(self.send_goal, self.home_idx)
                                self.looping = False
                                self.one_shot_mode = False
                            elif c == '2':
                                self.get_logger().info("串口收到2，恢复循环")
                                self.looping = True
                                self.one_shot_mode = False
                                rclpy.get_default_context().call_soon_threadsafe(self.loop_nav)
                            elif c == '3':
                                self.get_logger().info("串口收到3，发送guard点")
                                rclpy.get_default_context().call_soon_threadsafe(self.send_goal, self.guard_idx)
                                self.looping = False
                                self.one_shot_mode = False
                            elif c == '4':
                                self.get_logger().info("串口收到4，执行一遍多点导航")
                                self.one_shot_mode = True
                                self.one_shot_idx = 0
                                self.looping = False
                                rclpy.get_default_context().call_soon_threadsafe(self.send_goal, self.order[0])
            except serial.SerialException as e:
                self.get_logger().error(f"串口通信错误: {str(e)}")
                break
            except Exception as e:
                self.get_logger().error(f"串口接收错误: {str(e)}")
                break

    def send_goal(self, idx):
        if self.sending:
            return
        wp = self.waypoints[idx]
        goal = NavigateToPose.Goal()
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
        self.get_logger().info(f'发送目标点: {wp["Name"]}')
        self._send_goal_future = self.nav_ac.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_resp_cb)
        self.sending = True
        self.retry_count = 0

    def goal_resp_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.retry_count += 1
            if self.retry_count < self.max_retry:
                self.get_logger().info(f'目标被拒绝，重试第{self.retry_count+1}次...')
                self.sending = False
                self.send_goal(self.home_idx if not self.looping else self.order[self.loop_idx])
            else:
                self.get_logger().info(f'目标连续{self.max_retry}次被拒绝，切换下一个点...')
                self.sending = False
                self.retry_count = 0
                if self.looping:
                    self.loop_idx = (self.loop_idx + 1) % len(self.order)
                    self.send_goal(self.order[self.loop_idx])
            return
        self.get_logger().info('目标已被接受，等待到达...')
        self._get_result_future = gh.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, fut):
        status = fut.result().status
        # 默认未到达
        self.status_flag = 0
        if status == 4:
            self.get_logger().info('到达')
            # home/guard/one_shot到达都发1，循环模式到达不发1
            if self.one_shot_mode:
                self.status_flag = 1
            elif not self.looping:
                self.status_flag = 1
        else:
            self.get_logger().info(f'导航失败，状态码: {status}')
        self.sending = False
        self.retry_count = 0
        if self.one_shot_mode:
            self.one_shot_idx += 1
            if self.one_shot_idx < len(self.order):
                self.send_goal(self.order[self.one_shot_idx])
            else:
                self.get_logger().info('一遍多点导航完成，停止。')
                self.one_shot_mode = False
                self.looping = False
                self.one_shot_idx = 0
        elif self.looping:
            self.loop_idx = (self.loop_idx + 1) % len(self.order)
            self.send_goal(self.order[self.loop_idx])

    def loop_nav(self):
        if self.looping and not self.sending:
            self.send_goal(self.order[self.loop_idx])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial_port', type=str, default='/dev/ttyUSB0', help='串口端口')
    parser.add_argument('--baud_rate', type=int, default=115200, help='串口波特率')
    parser.add_argument('--yaml', type=str, default='waypoints.yaml', help='路径点yaml文件')
    parser.add_argument('--home', type=int, default=1, help='home点编号（从1开始）')
    parser.add_argument('--order', type=int, nargs='+', default=[1,3,5], help='循环点编号')
    parser.add_argument('--guard', type=int, default=2, help='guard点编号（从1开始）')
    parser.add_argument('--force_loop', action='store_true', help='强制开启循环模式')
    args = parser.parse_args()
    rclpy.init()
    node = GameNode(
        serial_port=args.serial_port,
        baud_rate=args.baud_rate,
        home_idx=args.home-1,
        order=[i-1 for i in args.order],
        guard_idx=args.guard-1,
        yaml_path=args.yaml,
        force_loop=args.force_loop
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点已停止")
    finally:
        if hasattr(node, 'ser') and node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
