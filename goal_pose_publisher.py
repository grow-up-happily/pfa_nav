import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class GoalPoseActionClient(Node):
    def __init__(self):
        super().__init__('goal_pose_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.has_sent = False
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        if self.has_sent:
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 4.023537284606579
        goal_msg.pose.pose.position.y = 7.868291729561693
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.6340028846927934
        goal_msg.pose.pose.orientation.w = 0.773330681016353

        self._action_client.wait_for_server()
        self.get_logger().info('已发送目标点到/navigate_to_pose')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.has_sent = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('目标已被接受，等待到达...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info('到达')
        else:
            self.get_logger().info(f'导航失败，状态码: {status}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # 可选：输出反馈信息
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseActionClient()
    rclpy.spin(node)

if __name__ == '__main__':
    main()