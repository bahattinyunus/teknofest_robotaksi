"""
EKF Localizer – Teknofest Robotaksi
=====================================
Fuses GNSS (position) and IMU (heading + angular velocity) measurements
using a 5-state Extended Kalman Filter.

State vector: [x, y, yaw, v, omega]
Observation model:
  GNSS  → [x, y]
  IMU   → [yaw, omega]
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import numpy as np
import math


def _quat_to_yaw(q: Quaternion) -> float:
    """Convert quaternion to yaw angle."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _gnss_to_xy(lat: float, lon: float, origin_lat: float, origin_lon: float):
    """Approximate GNSS to local Cartesian (ENU) assuming flat Earth."""
    R = 6_371_000.0
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    x = R * dlon * math.cos(math.radians(origin_lat))
    y = R * dlat
    return x, y


class EKFLocalizer(Node):
    """EKF-based vehicle localizer node."""

    DT = 0.05  # 20 Hz prediction

    def __init__(self):
        super().__init__('ekf_localizer')

        # State & covariance
        self._x = np.zeros(5)          # [x, y, yaw, v, omega]
        self._P = np.eye(5) * 1.0

        # Process noise
        q_diag = [0.1, 0.1, 0.05, 0.5, 0.1]
        self._Q = np.diag(q_diag)

        # Measurement noise – GNSS
        self._R_gnss = np.diag([1.0, 1.0])   # m²
        # Measurement noise – IMU
        self._R_imu = np.diag([0.02, 0.05])  # rad², (rad/s)²

        # GNSS origin (set on first fix)
        self._origin: tuple[float, float] | None = None

        # Publishers / subscribers
        self._odom_pub = self.create_publisher(Odometry, '/localization/odom', 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self._gnss_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_timer(self.DT, self._predict)

        self.get_logger().info('📍  EKF Localizer Active (5-state)')

    # ── Prediction ─────────────────────────────────────────────────────────

    def _predict(self):
        v, omega, yaw = self._x[3], self._x[4], self._x[2]
        dt = self.DT

        # State transition (bicycle kinematic model)
        F = np.eye(5)
        F[0, 2] = -v * math.sin(yaw) * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] = v * math.cos(yaw) * dt
        F[1, 3] = math.sin(yaw) * dt
        F[2, 4] = dt

        self._x[0] += v * math.cos(yaw) * dt
        self._x[1] += v * math.sin(yaw) * dt
        self._x[2] += omega * dt
        # v and omega remain constant between measurements

        self._P = F @ self._P @ F.T + self._Q
        self._publish()

    # ── GNSS Update ────────────────────────────────────────────────────────

    def _gnss_cb(self, msg: NavSatFix):
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
            self.get_logger().info(f'GNSS origin set: {self._origin}')
            return

        z_x, z_y = _gnss_to_xy(msg.latitude, msg.longitude, *self._origin)
        z = np.array([z_x, z_y])
        H = np.zeros((2, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        self._update(z, H, self._R_gnss)

    # ── IMU Update ─────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        yaw = _quat_to_yaw(msg.orientation)
        omega = msg.angular_velocity.z
        z = np.array([yaw, omega])
        H = np.zeros((2, 5))
        H[0, 2] = 1.0
        H[1, 4] = 1.0
        self._update(z, H, self._R_imu)

    # ── Generic EKF update ─────────────────────────────────────────────────

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray):
        y = z - H @ self._x
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(5) - K @ H) @ self._P

    # ── Publish ────────────────────────────────────────────────────────────

    def _publish(self):
        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self._x[0]
        msg.pose.pose.position.y = self._x[1]
        # Convert yaw back to quaternion (z-only rotation)
        yaw = self._x[2]
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.twist.twist.linear.x = self._x[3]
        msg.twist.twist.angular.z = self._x[4]
        self._odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
