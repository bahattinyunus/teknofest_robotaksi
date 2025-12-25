import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Image, '/perception/obstacles', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Obstacle Detector Node Started 🚀')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Dummy processing: Draw a rectangle representing detection
        # In a real scenario, this would be replaced by YOLOv8 inference
        cv2.rectangle(cv_image, (100, 100), (300, 300), (0, 0, 255), 2)
        cv2.putText(cv_image, "Obstacle Detected", (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher_.publish(out_msg)
        except Exception as e:
             self.get_logger().error(f'Could not publish image: {e}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector = ObstacleDetector()
    rclpy.spin(obstacle_detector)
    obstacle_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
