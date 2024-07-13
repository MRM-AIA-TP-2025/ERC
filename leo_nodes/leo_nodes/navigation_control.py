import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

class GoalSetterNode(Node):
    def __init__(self):
        super().__init__('goal_setter_node')
        
        # List of goals defined as arrays
        self.goals = [
            {"position": {"x": -12.085, "y": 9.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -7.0, "y": 10.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
            {"position": {"x": -4.2538, "y": 10.7378, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.154, "w": 1.0}},
        ]
        
        self.current_goal_index = 0
        
        # Publisher for sending goals to Nav2
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to send goals (for demonstration, you can trigger based on events or conditions)
        self.timer = self.create_timer(5.0, self.send_next_goal)
        
    def send_next_goal(self):
        self.check_goal_reached()
        if self.current_goal_index < len(self.goals):
            goal = self.goals[self.current_goal_index]
            
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # Assuming goals are in the 'map' frame
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
            
            goal = self.goals[current_goal_index]  # Get the last sent goal
            
            goal_x = goal['position']['x']
            goal_y = goal['position']['y']
            goal_z = goal['position']['z']
            
            # Calculate distance or use a threshold to determine if goal is reached
            distance_to_goal = ((current_x - goal_x)**2 + (current_y - goal_y)**2 + (current_z - goal_z)**2)**0.5
            
            if distance_to_goal < 1:  # Adjust threshold as needed
                self.get_logger().info('Goal reached!')
                self.current_goal_index += 1
        
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')

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
