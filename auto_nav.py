#!/usr/bin/env python3

# 自动循环导航 - 从文件读取目标点并自动循环导航
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
import yaml
import argparse
import time


class AutoNavNode(Node):
    def __init__(self, yaml_path, order, through_all_waypoints=False):
        super().__init__('auto_nav_node')
        
        # 加载航点
        self.waypoints = self.load_waypoints(yaml_path)
        self.get_logger().info(f"已从 {yaml_path} 加载 {len(self.waypoints)} 个航点")
        
        self.through_all_waypoints = through_all_waypoints

        if self.through_all_waypoints:
            self.targets = list(range(len(self.waypoints)))
            self.get_logger().info("已启用整条路线模式：将一次性规划并经过 YAML 中的全部航点")
        else:
            # 将目标点编号转换为索引（从1开始转为从0开始）
            self.targets = [idx - 1 for idx in order]

        # 验证目标点索引
        for i, idx in enumerate(self.targets):
            if 0 <= idx < len(self.waypoints):
                wp_name = self.waypoints[idx].get('Name', f'Waypoint_{idx+1}')
                if self.through_all_waypoints:
                    self.get_logger().info(f"路线点 {i+1}: 编号 {idx + 1} ({wp_name})")
                else:
                    self.get_logger().info(f"目标点 {i+1}: 编号 {order[i]} ({wp_name})")
            else:
                invalid_id = idx + 1 if self.through_all_waypoints else order[i]
                self.get_logger().error(f"目标点编号 {invalid_id} 超出范围 [1-{len(self.waypoints)}]")
                raise ValueError(f"无效的目标点编号: {invalid_id}")
        
        if not self.targets:
            self.get_logger().error("未加载到任何目标点，退出")
            raise ValueError("目标点列表为空")
        
        # 导航客户端
        self.nav_ac = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')
        self.nav_through_poses_ac = ActionClient(
            self, NavigateThroughPoses, '/red_standard_robot1/navigate_through_poses'
        )
        
        # 循环导航状态
        self.current_idx = 0
        self.sending = False
        self.max_retry = 3
        self.retry_count = 0
        
        # 启动导航循环定时器
        self.timer = self.create_timer(1.0, self.navigation_loop)
        self.get_logger().info("自动循环导航节点已启动")

    def load_waypoints(self, yaml_path):
        """从 YAML 文件加载所有航点"""
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        waypoints = []
        for i in range(1, data['Waypoints_Num'] + 1):
            waypoints.append(data[f'Waypoint_{i}'])
        return waypoints

    def load_targets_from_file(self, file_path):
        """从文件读取目标点索引列表
        
        文件格式：每行一个目标点编号（从1开始），支持注释（#开头）
        示例：
        # 这是注释
        1
        3
        5
        2
        """
        targets = []
        try:
            with open(file_path, 'r') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    # 跳过空行和注释
                    if not line or line.startswith('#'):
                        continue
                    try:
                        # 解析目标点编号（从1开始，需要转换为索引）
                        idx = int(line) - 1
                        if 0 <= idx < len(self.waypoints):
                            targets.append(idx)
                            wp_name = self.waypoints[idx].get('Name', f'Waypoint_{idx+1}')
                            self.get_logger().info(
                                f"第{line_num}行: 目标点 {line} -> 索引 {idx} ({wp_name})"
                            )
                        else:
                            self.get_logger().warn(
                                f"第{line_num}行: 目标点编号 {line} 超出范围 [1-{len(self.waypoints)}]，已跳过"
                            )
                    except ValueError:
                        self.get_logger().warn(f"第{line_num}行: 无效的目标点编号 '{line}'，已跳过")
            
            self.get_logger().info(f"从 {file_path} 成功加载 {len(targets)} 个目标点")
        except FileNotFoundError:
            self.get_logger().error(f"目标点文件不存在: {file_path}")
        except Exception as e:
            self.get_logger().error(f"读取目标点文件失败: {e}")
        
        return targets

    def send_goal(self, waypoint_idx):
        """发送导航目标点"""
        if self.sending:
            return
        
        goal = NavigateToPose.Goal()
        goal.pose = self.build_pose_stamped(waypoint_idx)
        
        self.nav_ac.wait_for_server()
        wp = self.waypoints[waypoint_idx]
        wp_name = wp.get('Name', f'Waypoint_{waypoint_idx+1}')
        self.get_logger().info(
            f"发送目标点 [{self.current_idx + 1}/{len(self.targets)}]: {wp_name}"
        )
        
        self._send_goal_future = self.nav_ac.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.sending = True
        self.retry_count = 0

    def send_route_goal(self):
        """一次性发送整条经过所有点的导航路线"""
        if self.sending:
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.build_pose_stamped(idx) for idx in self.targets]

        self.nav_through_poses_ac.wait_for_server()
        wp_names = [
            self.waypoints[idx].get('Name', f'Waypoint_{idx+1}')
            for idx in self.targets
        ]
        self.get_logger().info(
            f"发送整条路线，共 {len(goal.poses)} 个点: {' -> '.join(wp_names)}"
        )

        self._send_goal_future = self.nav_through_poses_ac.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.route_goal_response_callback)
        self.sending = True
        self.retry_count = 0

    def build_pose_stamped(self, waypoint_idx):
        """根据 waypoint 索引构造 PoseStamped"""
        wp = self.waypoints[waypoint_idx]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wp['Pos_x'])
        pose.pose.position.y = float(wp['Pos_y'])
        pose.pose.position.z = float(wp['Pos_z'])
        pose.pose.orientation.x = float(wp['Ori_x'])
        pose.pose.orientation.y = float(wp['Ori_y'])
        pose.pose.orientation.z = float(wp['Ori_z'])
        pose.pose.orientation.w = float(wp['Ori_w'])
        return pose

    def goal_response_callback(self, future):
        """处理导航目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.retry_count += 1
            if self.retry_count < self.max_retry:
                self.get_logger().warn(f"目标被拒绝，重试第 {self.retry_count} 次...")
                self.sending = False
                self.send_goal(self.targets[self.current_idx])
            else:
                self.get_logger().error(
                    f"目标连续 {self.max_retry} 次被拒绝，切换到下一个点..."
                )
                self.sending = False
                self.retry_count = 0
                self.current_idx = (self.current_idx + 1) % len(self.targets)
            return
        
        self.get_logger().info('目标已被接受，等待到达...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def route_goal_response_callback(self, future):
        """处理整条路线导航响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.retry_count += 1
            if self.retry_count < self.max_retry:
                self.get_logger().warn(f"整条路线目标被拒绝，重试第 {self.retry_count} 次...")
                self.sending = False
                self.send_route_goal()
            else:
                self.get_logger().error(
                    f"整条路线连续 {self.max_retry} 次被拒绝，等待下一轮重发..."
                )
                self.sending = False
                self.retry_count = 0
            return

        self.get_logger().info('整条路线目标已被接受，等待执行完成...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.route_result_callback)

    def result_callback(self, future):
        """处理导航结果"""
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ 成功到达目标点')
            # 单目标点模式：到达后结束
            if len(self.targets) == 1:
                self.get_logger().info('单目标点模式，导航完成，退出')
                self.timer.cancel()
                rclpy.shutdown()
                return
            time.sleep(0.5)
        else:
            self.get_logger().warn(f'✗ 导航失败，状态码: {status}')

        self.sending = False
        self.retry_count = 0

        # 切换到下一个目标点
        self.current_idx = (self.current_idx + 1) % len(self.targets)
        next_wp_name = self.waypoints[self.targets[self.current_idx]].get(
            'Name', f'Waypoint_{self.targets[self.current_idx]+1}'
        )
        self.get_logger().info(f"准备导航到下一个点: {next_wp_name}")

    def route_result_callback(self, future):
        """处理整条路线执行结果"""
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✓ 已按顺序通过整条路线中的所有目标点')
        else:
            self.get_logger().warn(f'✗ 整条路线导航失败，状态码: {status}')

        self.sending = False
        self.retry_count = 0
        time.sleep(0.5)
        self.get_logger().info('准备重新发送整条路线')

    def navigation_loop(self):
        """循环导航定时器回调"""
        if not self.sending:
            if self.through_all_waypoints:
                self.send_route_goal()
            else:
                self.send_goal(self.targets[self.current_idx])


def main():
    parser = argparse.ArgumentParser(description='自动循环导航 - 从文件读取目标点并循环导航')
    parser.add_argument(
        '--yaml', 
        type=str, 
        default='waypoints.yaml', 
        help='航点 YAML 文件路径'
    )
    parser.add_argument(
        '--order',
        type=int,
        nargs='+',
        default=[1],
        help='循环点编号列表（从1开始）'
    )
    parser.add_argument(
        '--through-all-waypoints',
        action='store_true',
        help='忽略 --order，一次性读取 YAML 中全部航点并通过 NavigateThroughPoses 规划经过所有点'
    )
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = AutoNavNode(
            yaml_path=args.yaml,
            order=args.order,
            through_all_waypoints=args.through_all_waypoints
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
