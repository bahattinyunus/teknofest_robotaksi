import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Path
import time
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self._output_limits = output_limits
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    def compute(self, measurement):
        now = time.time()
        dt = now - self._last_time
        if dt <= 0.0: dt = 1e-3
        error = self.setpoint - measurement
        self._integral += error * dt
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * (error - self._prev_error) / dt)
        lower, upper = self._output_limits
        if lower is not None: output = max(lower, output)
        if upper is not None: output = min(upper, output)
        self._prev_error = error
        self._last_time = now
        return output

class StanleyController:
    """Stanley Controller for lateral vehicle control."""
    def __init__(self, k=0.5):
        self.k = k # Control gain

    def compute(self, v, cte, yaw_error):
        # Steering angle = yaw_error + arctan(k * cte / v)
        v = max(v, 0.1) # Avoid division by zero
        steering = yaw_error + np.arctan2(self.k * cte, v)
        return np.clip(steering, -0.5, 0.5) # Limit steering to ~30 degrees

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('robotaksi_controller')
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        
        self.pid = PID(self.get_parameter('kp').value, self.get_parameter('ki').value,
                       self.get_parameter('kd').value, output_limits=(-1.0, 1.0))
        self.stanley = StanleyController(k=0.5)
        
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/planning/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/sensor/velocity', self.velocity_callback, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info('Stanley + PID Controller Stack Active 🛡️')

    def path_callback(self, msg):
        # In a real scenario, calculate CTE and Yaw Error from Path msg
        # Here we simulate values for demonstration
        self.target_velocity = 5.0 # Set a base target speed

    def velocity_callback(self, msg):
        self.current_velocity = msg.data

    def control_loop(self):
        # Longitudinal Control
        throttle = self.pid.compute(self.current_velocity)
        
        # Lateral Control (Stanley)
        # Dummy CTE and Yaw Error values (would come from Path/Localization)
        cte = 0.1 
        yaw_err = 0.05
        steering = self.stanley.compute(self.current_velocity, cte, yaw_err)
        
        # Velocity Profiling: Slow down if CTE is high
        if abs(cte) > 0.5:
            self.pid.setpoint = self.target_velocity * 0.5
        else:
            self.pid.setpoint = self.target_velocity

        twist = Twist()
        twist.linear.x = float(throttle)
        twist.angular.z = float(steering)
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
