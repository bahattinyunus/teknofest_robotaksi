import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.publisher_ = self.create_publisher(Path, '/planning/global_path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Global Planner Node Started 🗺️')

    def timer_callback(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create a dummy straight line path for testing
        # In a real scenario, this would use A* or Dijkstra on a map
        for i in range(10):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
            
        self.publisher_.publish(msg)
        self.get_logger().info('Published global path')

def main(args=None):
    rclpy.init(args=args)
    gp = GlobalPlanner()
    rclpy.spin(gp)
    gp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
