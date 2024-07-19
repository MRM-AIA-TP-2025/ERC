import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.publisher = self.create_publisher(Twist, 'gazebo/controllers/diff_drive/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_relay = CmdVelRelay()
    rclpy.spin(cmd_vel_relay)
    cmd_vel_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
