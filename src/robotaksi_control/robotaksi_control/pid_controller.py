import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time

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
        if dt <= 0.0:
            dt = 1e-3

        error = self.setpoint - measurement
        
        # Proportional
        p = self.kp * error
        
        # Integral with Anti-Windup (Basic clamping)
        self._integral += error * dt
        i = self.ki * self._integral
        
        # Derivative
        d = self.kd * (error - self._prev_error) / dt
        
        output = p + i + d
        
        # Apply output limits
        lower, upper = self._output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
            
        self._prev_error = error
        self._last_time = now
        
        return output

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        
        self.pid = PID(
            self.get_parameter('kp').value,
            self.get_parameter('ki').value,
            self.get_parameter('kd').value,
            output_limits=(-1.0, 1.0) # Normalized throttle/brake
        )
        
        self.current_velocity = 0.0
        
        self.publisher_ = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.target_sub = self.create_subscription(Float32, '/control/target_speed', self.target_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/sensor/velocity', self.velocity_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.get_logger().info('Robust PID Controller Node Active 🛡️')

    def target_callback(self, msg):
        self.pid.setpoint = msg.data

    def velocity_callback(self, msg):
        self.current_velocity = msg.data

    def control_loop(self):
        control_signal = self.pid.compute(self.current_velocity)
        
        twist = Twist()
        twist.linear.x = control_signal
        twist.angular.z = 0.0 # Lateral control not implemented in this node
        
        self.publisher_.publish(twist)
        # self.get_logger().info(f'Target: {self.pid.setpoint:.2f} | Current: {self.current_velocity:.2f} | Output: {control_signal:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
