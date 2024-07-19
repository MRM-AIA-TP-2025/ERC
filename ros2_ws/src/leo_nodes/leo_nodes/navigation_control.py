import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from std_msgs.msg import Empty, Int32

class GoalSetterNode(Node):
    def __init__(self):
        super().__init__('goal_setter_node')
       
        # List of goals
        self.goals = [
            {"position": {"x": -12.085, "y": 9.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -9.689, "y": 9.105, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -7.246, "y": 9.765, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -4.2538, "y": 10.7378, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -3.003, "y": 6.586, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -0.328, "y": 4.391, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": 3.435, "y": 4.614, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": 5.929, "y": 4.032, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": 5.845, "y": 5.306, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.7, "w": 0.713}},
            {"position": {"x": 6.425, "y": 6.425, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.807, "w": 0.589}},
            {"position": {"x": 6.518, "y": 8.456, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.687,"w": 0.726}},
            {"position": {"x": 8.376, "y": 11.457, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.687,"w": 0.726}},
            {"position": {"x": 5.742, "y": 4.035, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.856,"w": 0.516}},
            {"position": {"x": -0.4238, "y": -6.9421, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.8832,"w": 0.4689}},
            {"position": {"x": -6.3938, "y": -11.45, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.8832,"w": 0.4689}},
            {"position": {"x": -2.169, "y": -2.884, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.610,"w": 0.792}},
            {"position": {"x": -1.44206, "y": 5.2585, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.95162 ,"w": 0.3073}},
            {"position": {"x": -17.0038, "y": 7.6178, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 1.0 ,"w": 0.0}},
        ]

        self.probe_drop_goals = [3, 11, 13, 14] 
        self.current_goal_index = 0
       
        # Publisher for sending goals to Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
       
        # Publisher for dropping probes
        self.drop_pub = self.create_publisher(Empty, 'probe_deployment_unit/drop', 10)
       
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_nav = False

        self.controller_sub = self.create_subscription(Int32, '/controller', self.controller_callback, 10)

        # Timer to send goals
        self.timer = self.create_timer(5.0, self.send_next_goal)
       
    def send_next_goal(self):
        if not self.start_nav:
            return

        self.check_goal_reached()
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
           
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map' 
            goal_msg.pose.position.x = goal['position']['x']
            goal_msg.pose.position.y = goal['position']['y']
            goal_msg.pose.position.z = goal['position']['z']
            goal_msg.pose.orientation.x = goal['orientation']['x']
            goal_msg.pose.orientation.y = goal['orientation']['y']
            goal_msg.pose.orientation.z = goal['orientation']['z']
            goal_msg.pose.orientation.w = goal['orientation']['w']
           
            self.goal_pub.publish(goal_msg)
           
            self.get_logger().info(f'Sending goal {self.current_goal_index + 1}: {goal}')
           
        else:
            self.get_logger().info('No more goals to send.')

    def check_goal_reached(self):
        try:
            # Get the transform from 'odom' to 'base_footprint' (current rover location)
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            # Check if the current location is near the goal
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_z = transform.transform.translation.z
           
            goal = self.goals[self.current_goal_index]  # Get the last sent goal
           
            goal_x = goal['position']['x']
            goal_y = goal['position']['y']
            goal_z = goal['position']['z']
           
            # Calculate distance
            distance_to_goal = ((current_x - goal_x)**2 + (current_y - goal_y)**2 + (current_z - goal_z)**2)**0.5
           
            if distance_to_goal < 1.8: 
                self.get_logger().info('Goal reached!')
               
                if self.current_goal_index in self.probe_drop_goals:
                    self.drop_probe()
               
                self.current_goal_index += 1
                if current_goal_index == len(self.goals):
                    self.start_nav = False
       
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')

    def drop_probe(self):
        self.get_logger().info('Dropping probe...')
        self.drop_pub.publish(Empty())

    def controller_callback(self,msg):
        if msg.data == -1:
            self.get_logger().info('Starting Autonomous Navigation!')
            self.start_nav = True
        else:
            self.current_goal_index = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = GoalSetterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()