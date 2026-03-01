"""
Lane Detector Node – Teknofest Robotaksi
=========================================
Implements a sliding-window Hough-transform lane detection pipeline for
real-time lane boundary estimation from monocular camera images.

Pipeline:
  1. BGR → Grayscale → Gaussian Blur
  2. Canny Edge Detection (adaptive thresholds)
  3. ROI masking (trapezoid)
  4. Hough Line Transform
  5. Left / Right lane classification & averaging
  6. Deviation (cross-track error) publication
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


def _canny(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    v = np.median(blur)
    lo = int(max(0, 0.67 * v))
    hi = int(min(255, 1.33 * v))
    return cv2.Canny(blur, lo, hi)


def _roi(image: np.ndarray) -> np.ndarray:
    h, w = image.shape[:2]
    mask = np.zeros_like(image)
    trap = np.array([[
        (int(w * 0.05), h),
        (int(w * 0.40), int(h * 0.55)),
        (int(w * 0.60), int(h * 0.55)),
        (int(w * 0.95), h),
    ]])
    cv2.fillPoly(mask, trap, 255)
    return cv2.bitwise_and(image, mask)


def _fit_line(lines):
    """Separate left/right lines and return averaged (slope, intercept) tuples."""
    left, right = [], []
    if lines is None:
        return None, None
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        if x2 == x1:
            continue
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        (left if slope < 0 else right).append((slope, intercept))

    def avg(group):
        if not group:
            return None
        return tuple(np.mean(group, axis=0))

    return avg(left), avg(right)


def _make_coords(image, line_params):
    if line_params is None:
        return None
    slope, intercept = line_params
    h = image.shape[0]
    y1 = h
    y2 = int(h * 0.55)
    if abs(slope) < 1e-6:
        return None
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return (x1, y1, x2, y2)


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self._cb, 10)
        self.img_pub = self.create_publisher(Image, '/perception/lane_viz', 10)
        self.cte_pub = self.create_publisher(Float32, '/perception/lane_cte', 10)
        self.get_logger().info('🛣️  Lane Detector Node Active')

    def _cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Bridge error: {exc}')
            return

        edges = _canny(frame)
        roi_edges = _roi(edges)
        lines = cv2.HoughLinesP(roi_edges, 2, np.pi / 180, 100,
                                minLineLength=40, maxLineGap=5)

        left_params, right_params = _fit_line(lines)
        left_line = _make_coords(frame, left_params)
        right_line = _make_coords(frame, right_params)

        viz = frame.copy()
        lane_center_x = frame.shape[1] // 2  # fallback
        for ln, color in [(left_line, (0, 255, 0)), (right_line, (0, 0, 255))]:
            if ln:
                cv2.line(viz, (ln[0], ln[1]), (ln[2], ln[3]), color, 6)

        if left_line and right_line:
            lane_center_x = (left_line[0] + right_line[0]) // 2
        elif left_line:
            lane_center_x = left_line[0] + 150
        elif right_line:
            lane_center_x = right_line[0] - 150

        img_center_x = frame.shape[1] // 2
        cte = float(img_center_x - lane_center_x) / (frame.shape[1] / 2)  # normalised [-1, 1]

        cte_msg = Float32()
        cte_msg.data = cte
        self.cte_pub.publish(cte_msg)

        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(viz, 'bgr8'))
        except Exception as exc:
            self.get_logger().error(f'Publish error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
