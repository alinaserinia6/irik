import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import numpy as np
from typing import List

class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Publishers
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/navigation/target_pose', 10)
        self.path_pub = self.create_publisher(
            Path, '/navigation/planned_path', 10)
        
        # Subscribers
        self.current_pose_sub = self.create_subscription(
            PoseStamped, '/vehicle/pose', self.pose_callback, 10)
        
        # Timer for navigation loop
        self.timer = self.create_timer(0.2, self.navigation_loop)  # 5Hz
        
        # Navigation state
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 1.0  # meters
        
        self.get_logger().info("Waypoint Navigator Node Started")

    def pose_callback(self, msg):
        self.current_pose = msg

    def add_waypoint(self, x: float, y: float, z: float):
        """Add waypoint to mission"""
        waypoint = Point()
        waypoint.x = x
        waypoint.y = y
        waypoint.z = z
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Added waypoint: ({x}, {y}, {z})")

    def navigation_loop(self):
        """Main navigation loop"""
        if self.current_pose is None or not self.waypoints:
            return
        
        # TODO: Implement navigation logic
        # 1. Check if current waypoint is reached
        # 2. Move to next waypoint
        # 3. Publish target pose
        pass

    def calculate_distance_to_waypoint(self, waypoint: Point) -> float:
        """Calculate distance to given waypoint"""
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.pose.position.x - waypoint.x
        dy = self.current_pose.pose.position.y - waypoint.y
        dz = self.current_pose.pose.position.z - waypoint.z
        
        return np.sqrt(dx**2 + dy**2 + dz**2)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

