#!/usr/bin/env python3

# 根据机器人血量自动切换导航点
# 血量 < 阈值 → 导航到第二个点；否则 → 导航到第一个点
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import serial
import threading
import yaml
import argparse


class HpNavNode(Node):
    def __init__(self, serial_port, baud_rate, yaml_path, point_normal, point_low_hp, hp_threshold):
        super().__init__('hp_nav_node')

        # 加载航点
        self.waypoints = self.load_waypoints(yaml_path)
        self.get_logger().info(f"已从 {yaml_path} 加载 {len(self.waypoints)} 个航点")

        # 两个目标点索引（从1转为从0）
        self.point_normal = point_normal - 1
        self.point_low_hp = point_low_hp - 1
        for idx, label in [(self.point_normal, '正常'), (self.point_low_hp, '低血量')]:
            if idx < 0 or idx >= len(self.waypoints):
                raise ValueError(f"{label}点编号超出范围 [1-{len(self.waypoints)}]")

        self.hp_threshold = hp_threshold
        self.current_hp = 600  # 默认满血
        self.current_target = self.point_normal

        self.get_logger().info(
            f"正常点: {self.waypoints[self.point_normal].get('Name', self.point_normal+1)}, "
            f"低血量点: {self.waypoints[self.point_low_hp].get('Name', self.point_low_hp+1)}, "
            f"血量阈值: {self.hp_threshold}"
        )

        # 串口
        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f"串口连接成功: {serial_port}")
        except Exception as e:
            self.get_logger().error(f"串口连接失败: {e}")

        # 导航客户端
        self.nav_ac = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')

        # 导航状态
        self.sending = False
        self.max_retry = 3
        self.retry_count = 0

        # 串口接收线程
        if self.ser:
            self.receiver_thread = threading.Thread(target=self.serial_receiver, daemon=True)
            self.receiver_thread.start()

        # 导航循环定时器
        self.timer = self.create_timer(1.0, self.navigation_loop)
        self.get_logger().info("血量导航节点已启动")

    def load_waypoints(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        wps = []
        for i in range(1, data['Waypoints_Num'] + 1):
            wps.append(data[f'Waypoint_{i}'])
        return wps

    def serial_receiver(self):
        """持续从串口读取血量数据"""
        buf = b''
        while rclpy.ok() and self.ser:
            try:
                if self.ser.in_waiting > 0:
                    buf += self.ser.read(self.ser.in_waiting)
                    # 按换行符拆分，取最新的完整行
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

    def navigation_loop(self):
        """根据血量决定目标点并导航"""
        # 根据血量选择目标点
        if self.current_hp < self.hp_threshold:
            desired_target = self.point_low_hp
        else:
            desired_target = self.point_normal

        # 目标点切换时取消当前导航
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
    parser = argparse.ArgumentParser(description='根据血量自动切换导航点')
    parser.add_argument('--serial_port', type=str, default='/dev/gimbal', help='串口端口')
    parser.add_argument('--baud_rate', type=int, default=115200, help='串口波特率')
    parser.add_argument('--yaml', type=str, default='waypoints.yaml', help='航点 YAML 文件路径')
    parser.add_argument('--point_normal', type=int, default=1, help='正常血量时的目标点编号(从1开始)')
    parser.add_argument('--point_low_hp', type=int, default=2, help='低血量时的目标点编号(从1开始)')
    parser.add_argument('--hp_threshold', type=int, default=200, help='血量阈值，低于此值切换到低血量点')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = HpNavNode(
            serial_port=args.serial_port,
            baud_rate=args.baud_rate,
            yaml_path=args.yaml,
            point_normal=args.point_normal,
            point_low_hp=args.point_low_hp,
            hp_threshold=args.hp_threshold,
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
