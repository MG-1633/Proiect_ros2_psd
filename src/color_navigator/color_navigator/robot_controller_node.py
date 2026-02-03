import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.color_sub = self.create_subscription(String, '/detected_color', self.color_callback, 10)
        
        self.state = 'SEARCHING'
        self.colors_seen = set()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.state == 'SEARCHING':
            self.searching()
        elif self.state == 'TURNING':
            self.turning()

    def searching(self):
        twist = Twist()
        twist.angular.z = 0.3  # Spin in place
        self.cmd_vel_pub.publish(twist)

    def turning(self):
        twist = Twist()
        twist.angular.z = 0.5  # Turn
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Turning 90 degrees...')
        self.state = 'SEARCHING'  # Transition back to searching after turning

    def color_callback(self, msg):
        color_name = msg.data
        if color_name not in self.colors_seen:
            self.colors_seen.add(color_name)
            self.get_logger().info(f'I see {color_name}!')
            self.state = 'TURNING'  # Transition to turning state

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()