import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.color_publisher = self.create_publisher(String, '/detected_color', 10)
        self.get_logger().info('Color Detector Node has been started.')

    def image_callback(self, msg):
        # Convert the ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges in HSV
        color_ranges = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'green': ((40, 100, 100), (80, 255, 255)),
            'blue': ((100, 100, 100), (140, 255, 255)),
            'yellow': ((20, 100, 100), (40, 255, 255))
        }

        detected_colors = {}

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area > 500:  # Area threshold to consider it a valid detection
                    detected_colors[color] = area

        if detected_colors:
            dominant_color = max(detected_colors, key=detected_colors.get)
            self.color_publisher.publish(String(data=dominant_color))
            self.get_logger().info(f'Detected color: {dominant_color}')

def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    rclpy.spin(color_detector_node)
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()