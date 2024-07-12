import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class NavigationController(Node):

    def __init__(self):
        super().__init__('navigation_controller')
        self.declare_parameter('goal1', [0.0, 0.0, 0.0])
        self.declare_parameter('goal2', [1.0, 1.0, 1.0])
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(5.0, self.send_goal)
        self.send_goal()

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        
        goal = self.get_parameter('goal1').get_parameter_value().double_array_value  # Change this to "goal2" to switch goals
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]
        goal_msg.pose.pose.orientation.w = goal[2]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal was successful')
        elif status == rclpy.action.GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted')
        elif status == rclpy.action.GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled')
        else:
            self.get_logger().error('Unknown result code')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
