import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('system_diagnostics')
        self.publisher_ = self.create_publisher(String, '/system/status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('System Diagnostics Node Active 📊')

    def timer_callback(self):
        stats = {
            "cpu_percent": psutil.cpu_percent(),
            "memory_percent": psutil.virtual_memory().percent,
            "system_load": psutil.getloadavg() if hasattr(psutil, 'getloadavg') else None,
            "status": "Operational"
        }
        
        msg = String()
        msg.data = json.dumps(stats)
        self.publisher_.publish(msg)
        
        if stats["cpu_percent"] > 90:
            self.get_logger().warn(f'Critical CPU Load: {stats["cpu_percent"]}%')

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
