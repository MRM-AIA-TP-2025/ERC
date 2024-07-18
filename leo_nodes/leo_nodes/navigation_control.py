import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('simple_navigator')

        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        self._waiting_for_server = True

        # Create a timer to periodically check for the action server (optional)
        self._server_check_timer = self.create_timer(0.5, self._check_action_server)

    def _check_action_server(self):
        if not self._waiting_for_server:
            return

        if self._action_client.wait_for_server(timeout_sec=1.0):
            self._waiting_for_server = False
            self.get_logger().info('"/navigate_to_pose" action server is now available')
            self._send_goal()

    def _send_goal(self):
        # Prompt user for goal coordinates and orientation
        try:
            x = float(input("Enter the x coordinate of the goal: "))
            y = float(input("Enter the y coordinate of the goal: "))
            z = float(input("Enter the z coordinate of the goal: "))
            w = float(input("Enter the w orientation of the goal: "))
            z = float(input("Enter the z orientation of the goal: "))
        except ValueError:
            self.get_logger().info('Invalid input. Please enter numeric values.')
            return

        # Define the goal pose
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = "map"  # Replace with your map frame ID
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = z
        goal_pose.pose.pose.orientation.w = w
        goal_pose.pose.pose.orientation.z = z

        send_goal_future = self._action_client.send_goal_async(goal_pose)

        try:
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal was rejected by the action server')
            else:
                self.get_logger().info('Goal accepted by the action server')
                self._wait_for_result(goal_handle)

        except KeyboardInterrupt:
            self.get_logger().info('Interrupted by user')
            send_goal_future.cancel()

    def _wait_for_result(self, goal_handle):
        while rclpy.ok():
            future = goal_handle.get_result_async()
            rclpy.spin_once(self)  # Spin to allow callbacks to run

            if future.done():
                try:
                    result = future.result()
                except RuntimeError as ex:
                    self.get_logger().info('Goal was aborted: {}'.format(ex))
                    break
                else:
                    if result.result:
                        self.get_logger().info('Goal succeeded!')
                    else:
                        self.get_logger().info('Goal failed with result: {}'.format(result.reason))
                    break

def main():
    rclpy.init()
    navigator = SimpleNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
