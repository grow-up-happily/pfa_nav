import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml

class NavLoop(Node):
    def __init__(self, wps, order, max_retry=3):
        super().__init__('nav_loop')
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.wps = wps
        self.order = order
        self.idx = 0
        self.sending = False
        self.max_retry = max_retry
        self.retry_count = 0
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        if self.sending or not self.order:
            return
        i = self.order[self.idx]
        wp = self.wps[i]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['Pos_x']
        goal.pose.pose.position.y = wp['Pos_y']
        goal.pose.pose.position.z = wp['Pos_z']
        goal.pose.pose.orientation.x = wp['Ori_x']
        goal.pose.pose.orientation.y = wp['Ori_y']
        goal.pose.pose.orientation.z = wp['Ori_z']
        goal.pose.pose.orientation.w = wp['Ori_w']
        self.ac.wait_for_server()
        self.get_logger().info(f'发送目标点: {wp["Name"]}（第{self.retry_count+1}次尝试）')
        self._send_goal_future = self.ac.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_resp_cb)
        self.sending = True

    def goal_resp_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.retry_count += 1
            if self.retry_count < self.max_retry:
                self.get_logger().info(f'目标被拒绝，重试第{self.retry_count+1}次...')
                self.sending = False
                self.send_goal()  # 继续尝试当前点
            else:
                self.get_logger().info(f'目标连续{self.max_retry}次被拒绝，切换下一个点...')
                self._next()
                self.sending = False
                self.retry_count = 0
                self.send_goal()  # 立即尝试下一个点
            return
        self.get_logger().info('目标已被接受，等待到达...')
        self._get_result_future = gh.get_result_async()
        self._get_result_future.add_done_callback(self.result_cb)

    def result_cb(self, fut):
        status = fut.result().status
        if status == 4:
            self.get_logger().info('到达')
        else:
            self.get_logger().info(f'导航失败，状态码: {status}')
        self._next()
        self.sending = False
        self.retry_count = 0

    def _next(self):
        self.idx = (self.idx + 1) % len(self.order)

    def feedback_cb(self, fb):
        pass

def load_wps(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    wps = []
    for i in range(1, data['Waypoints_Num']+1):
        wps.append(data[f'Waypoint_{i}'])
    return wps

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--yaml', type=str, default='waypoints.yaml', help='路径点yaml文件')
    parser.add_argument('--order', type=int, nargs='+', required=True, help='自定义循环顺序（点的编号，从1开始）')
    parser.add_argument('--max_retry', type=int, default=3, help='每个点最大重试次数')
    args = parser.parse_args()
    wps = load_wps(args.yaml)
    order = [i-1 for i in args.order]
    rclpy.init()
    node = NavLoop(wps, order, max_retry=args.max_retry)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

# python3 nav.py --order 1 3 5 --max_retry 3
# max_retry 如果连续max_retry次失败，则切换下一个点