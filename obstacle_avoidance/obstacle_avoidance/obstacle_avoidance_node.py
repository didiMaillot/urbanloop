import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__("laser_scan_subscriber")
        
        # Use Best Effort QoS to match the publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Avoid QoS mismatch issues
            depth=10  # Queue size
        )

        self.subscription = self.create_subscription(
            LaserScan,
            "/shuttle_3/ouster/scan",
            self.listener_callback,
            qos_profile
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Subscribed to /shuttle_3/ouster/scan with Best Effort QoS")

    def listener_callback(self, msg):
        """ Callback function to process received LaserScan messages. """
        # Filter the ranges to remove invalid readings
        filtered_ranges = [
            r for r in msg.ranges 
            if msg.range_min <= r <= msg.range_max and r != float('inf')
        ]
        
        if filtered_ranges:
            min_range = min(filtered_ranges)  # Find the closest object

            if min_range < 1.0:
                self.get_logger().warn(f"⚠️ Obstacle detected! Closest object at {min_range:.2f} meters.")
            else:
                self.get_logger().info(f"✅ Clear path. Closest object at {min_range:.2f} meters.")
        else:
            self.get_logger().info("No valid range readings available.")


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSubscriber()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

