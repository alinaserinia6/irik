import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus
from std_msgs.msg import String
import json

class MAVLinkBridgeNode(Node):
    def __init__(self):
        super().__init__('mavlink_bridge')
        
        # Publishers for processed data
        self.system_status_pub = self.create_publisher(
            String, '/system/status', 10)
        
        # Subscribers to PX4 topics
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, 10)
        self.battery_status_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_callback, 10)
        
        # Timer for status publishing
        self.timer = self.create_timer(1.0, self.publish_system_status)  # 1Hz
        
        # System state
        self.vehicle_status = None
        self.local_position = None
        self.battery_status = None
        
        self.get_logger().info("MAVLink Bridge Node Started")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def local_position_callback(self, msg):
        self.local_position = msg

    def battery_callback(self, msg):
        self.battery_status = msg

    def publish_system_status(self):
        """Publish consolidated system status"""
        if not all([self.vehicle_status, self.local_position, self.battery_status]):
            return
        
        # TODO: Create comprehensive system status message
        status_data = {
            'timestamp': self.get_clock().now().nanoseconds,
            'armed': bool(self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED),
            'flight_mode': self.vehicle_status.nav_state,
            'position': {
                'x': self.local_position.x,
                'y': self.local_position.y,
                'z': self.local_position.z
            },
            'battery_remaining': self.battery_status.remaining * 100.0
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.system_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MAVLinkBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

