import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/control/target_speed',
            self.listener_callback,
            10)
        self.get_logger().info('PID Controller Node Started 🎛️')

    def listener_callback(self, msg):
        target_speed = msg.data
        twist = Twist()
        
        # Simple proportional control dummy
        # In a real implementation, you would calculate errors and apply PID gains
        twist.linear.x = target_speed 
        twist.angular.z = 0.0 # Drive straight
        
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing cmd_vel: {twist.linear.x} m/s')

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
