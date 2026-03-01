"""
Behavior State Machine – Teknofest Robotaksi
=============================================
A finite-state machine (FSM) that coordinates high-level driving behaviour.

States:
  IDLE        – vehicle is stationary, awaiting mission start
  MISSION     – following a global path under normal conditions
  OBSTACLE    – halting / replanning around a detected obstacle
  EMERGENCY   – system fault; safe stop commanded

Transitions are triggered by ROS 2 topics, enabling clean decoupling of
the perception and control layers.
"""

from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    MISSION = auto()
    OBSTACLE = auto()
    EMERGENCY = auto()


class BehaviorFSM(Node):

    def __init__(self):
        super().__init__('behavior_fsm')
        self._state = State.IDLE

        # Publishers
        self._state_pub = self.create_publisher(String, '/behavior/state', 10)
        self._cmd_pub = self.create_publisher(Twist, '/behavior/cmd_vel', 10)

        # Subscribers
        self.create_subscription(Bool, '/mission/start', self._on_start, 10)
        self.create_subscription(Bool, '/mission/stop', self._on_stop, 10)
        self.create_subscription(Bool, '/perception/obstacle_detected',
                                 self._on_obstacle, 10)
        self.create_subscription(Bool, '/system/fault', self._on_fault, 10)
        self.create_subscription(Bool, '/perception/obstacle_cleared',
                                 self._on_cleared, 10)

        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('🧠  Behavior FSM Active – State: IDLE')

    # ── Event handlers ─────────────────────────────────────────────────────

    def _on_start(self, msg: Bool):
        if msg.data and self._state == State.IDLE:
            self._transition(State.MISSION)

    def _on_stop(self, msg: Bool):
        if msg.data:
            self._transition(State.IDLE)

    def _on_obstacle(self, msg: Bool):
        if msg.data and self._state == State.MISSION:
            self._transition(State.OBSTACLE)

    def _on_cleared(self, msg: Bool):
        if msg.data and self._state == State.OBSTACLE:
            self._transition(State.MISSION)

    def _on_fault(self, msg: Bool):
        if msg.data:
            self._transition(State.EMERGENCY)

    # ── State machine tick ─────────────────────────────────────────────────

    def _tick(self):
        cmds = {
            State.IDLE:      Twist(),              # zero velocity
            State.MISSION:   self._mission_cmd(),
            State.OBSTACLE:  Twist(),              # stop while replanning
            State.EMERGENCY: Twist(),              # hard stop
        }
        self._cmd_pub.publish(cmds[self._state])
        state_msg = String()
        state_msg.data = self._state.name
        self._state_pub.publish(state_msg)

    def _mission_cmd(self) -> Twist:
        """Placeholder: real missions read from the planning topic."""
        twist = Twist()
        twist.linear.x = 2.0   # m/s forward
        return twist

    def _transition(self, new_state: State):
        self.get_logger().info(f'FSM: {self._state.name} → {new_state.name}')
        self._state = new_state


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
