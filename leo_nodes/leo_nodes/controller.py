import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(Int32, '/controller', 10)
        self.get_input_and_publish()

    def get_input_and_publish(self):
        while rclpy.ok():
            user_input = input('Input \'-1\' to Start Navigation or CheckPoint to head towards: ')
            try:
                user_input = int(user_input)
                if user_input == -1:
                    output = -1
                elif user_input == 1:
                    output = 0
                elif user_input == 2:
                    output = 4
                elif user_input == 3:
                    output = 12
                else:
                    output = 15
                    user_input = 4
                if output == -1:
                    self.get_logger().info('Starting Autonomous Navigation')
                else:
                    self.get_logger().info(f'Heading towards {user_input}')
                msg = Int32()
                msg.data = output
                self.publisher_.publish(msg)
                
            except ValueError:
                self.get_logger().info('Invalid input')
            
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
