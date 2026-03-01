"""
Local Planner Node – Teknofest Robotaksi
=========================================
Implements a simplified Dynamic Window Approach (DWA) for real-time
obstacle avoidance and velocity profile generation.

The DWA samples (v, omega) pairs within the dynamic window, simulates
short-horizon trajectories, and selects the best command based on:
  score = goal_score + speed_score - obstacle_score
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
import numpy as np
import math


# ────────────────── DWA Helpers ──────────────────

class RobotState:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.omega = 0.0


class DWAConfig:
    max_v: float = 5.0          # m/s
    min_v: float = 0.0
    max_omega: float = 1.0      # rad/s
    max_accel: float = 1.0
    max_alpha: float = 2.0
    dt: float = 0.1
    predict_time: float = 2.0
    goal_cost_gain: float = 0.15
    speed_cost_gain: float = 1.0
    obs_cost_gain: float = 1.0
    v_resolution: float = 0.05
    omega_resolution: float = 0.1


def _calc_dw(state: RobotState, cfg: DWAConfig):
    v_min = max(cfg.min_v, state.v - cfg.max_accel * cfg.dt)
    v_max = min(cfg.max_v, state.v + cfg.max_accel * cfg.dt)
    w_min = max(-cfg.max_omega, state.omega - cfg.max_alpha * cfg.dt)
    w_max = min(cfg.max_omega, state.omega + cfg.max_alpha * cfg.dt)
    return v_min, v_max, w_min, w_max


def _simulate(state: RobotState, v: float, omega: float, cfg: DWAConfig):
    traj = []
    s = RobotState()
    s.x, s.y, s.yaw, s.v, s.omega = state.x, state.y, state.yaw, v, omega
    t = 0.0
    while t <= cfg.predict_time:
        s.x += v * math.cos(s.yaw) * cfg.dt
        s.y += v * math.sin(s.yaw) * cfg.dt
        s.yaw += omega * cfg.dt
        traj.append((s.x, s.y))
        t += cfg.dt
    return traj


def _dwa_control(state: RobotState, goal, obstacles, cfg: DWAConfig):
    v_min, v_max, w_min, w_max = _calc_dw(state, cfg)
    best_score = -float('inf')
    best_v, best_w = 0.0, 0.0

    for v in np.arange(v_min, v_max, cfg.v_resolution):
        for w in np.arange(w_min, w_max, cfg.omega_resolution):
            traj = _simulate(state, v, w, cfg)
            end = traj[-1]

            goal_score = -math.hypot(end[0] - goal[0], end[1] - goal[1])
            speed_score = v / cfg.max_v
            obs_score = 0.0
            for ox, oy in obstacles:
                min_d = min(math.hypot(px - ox, py - oy) for px, py in traj)
                if min_d < 0.5:
                    obs_score = -1e6
                    break

            score = (cfg.goal_cost_gain * goal_score +
                     cfg.speed_cost_gain * speed_score +
                     cfg.obs_cost_gain * obs_score)
            if score > best_score:
                best_score = score
                best_v, best_w = v, w

    return best_v, best_w


# ────────────────── ROS Node ──────────────────

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        self.cfg = DWAConfig()
        self.state = RobotState()
        self.goal = (50.0, 0.0)
        self.obstacles: list[tuple[float, float]] = []

        self.cmd_pub = self.create_publisher(Twist, '/planning/cmd_vel_local', 10)
        self.path_sub = self.create_subscription(Path, '/planning/global_path',
                                                  self._path_cb, 10)
        self.vel_sub = self.create_subscription(Float32, '/sensor/velocity',
                                                 self._vel_cb, 10)
        self.timer = self.create_timer(self.cfg.dt, self._control_step)
        self.get_logger().info('⚡  DWA Local Planner Active')

    def _path_cb(self, msg: Path):
        if msg.poses:
            pose: PoseStamped = msg.poses[-1]
            self.goal = (pose.pose.position.x, pose.pose.position.y)

    def _vel_cb(self, msg: Float32):
        self.state.v = msg.data

    def _control_step(self):
        v, omega = _dwa_control(self.state, self.goal, self.obstacles, self.cfg)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
