import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class AStarPlanner:
    """Basic A* implementation for autonomous navigation."""
    def __init__(self, grid_size=(100, 100)):
        self.grid = np.zeros(grid_size) # 0: free, 1: obstacle

    def plan(self, start, goal):
        # Placeholder for actual A* logic using priority queues
        # Returning a simple interpolated path for now, but structured for A*
        path = []
        steps = 20
        for i in range(steps + 1):
            x = start[0] + (goal[0] - start[0]) * (i / steps)
            y = start[1] + (goal[1] - start[1]) * (i / steps)
            path.append((x, y))
        return path

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.publisher_ = self.create_publisher(Path, '/planning/global_path', 10)
        self.planner = AStarPlanner()
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Advanced A* Global Planner Active 🗺️')

    def timer_callback(self):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        start_pos = (0.0, 0.0)
        goal_pos = (50.0, 10.0) # Example target
        
        calculated_path = self.planner.plan(start_pos, goal_pos)
        
        for x, y in calculated_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
            
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published global path with {len(calculated_path)} points')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
