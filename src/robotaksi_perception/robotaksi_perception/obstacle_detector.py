import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectionModel:
    """Modular class for object detection logic."""
    def __init__(self, mode='adaptive'):
        self.mode = mode
        # In a real scenario, load YOLOv8 weights here: 
        # self.model = YOLO('yolov8n.pt')

    def detect(self, image):
        if self.mode == 'yolo':
            # Placeholder for actual YOLO inference
            return [{"label": "obstacle", "bbox": (100, 100, 200, 200), "conf": 0.95}]
        else:
            # Advanced fallback: Adaptive thresholding for obstacle segmentation
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                          cv2.THRESH_BINARY_INV, 11, 2)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detections = []
            for cnt in contours:
                if cv2.contourArea(cnt) > 500: # Filter small noise
                    x, y, w, h = cv2.boundingRect(cnt)
                    detections.append({"label": "obstacle", "bbox": (x, y, w, h), "conf": 0.70})
            return detections

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/perception/obstacles', 10)
        self.bridge = CvBridge()
        self.model = DetectionModel(mode='adaptive')
        self.get_logger().info('Advanced Obstacle Detector Node Started 🚀')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        detections = self.model.detect(cv_image)

        for det in detections:
            x, y, w, h = det['bbox']
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{det['label']} {det['conf']:.2f}", 
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher_.publish(out_msg)
        except Exception as e:
             self.get_logger().error(f'Could not publish image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
