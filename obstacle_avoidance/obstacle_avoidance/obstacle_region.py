import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
import math


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
        
        # Get the total number of readings in the laser scan
        num_readings = len(msg.ranges)
        
        # Filter the ranges to remove invalid readings
        filtered_ranges = [
            r for r in msg.ranges 
            if msg.range_min <= r <= msg.range_max and r != float('inf')
        ]
        
        if filtered_ranges:
            # Divide the range readings into regions
            top_left = filtered_ranges[int(num_readings * 0.125):int(num_readings * 0.25)]  # 0° to 45°
            top = filtered_ranges[int(num_readings * 0.25):int(num_readings * 0.375)]  # 45° to 90°
            top_right = filtered_ranges[int(num_readings * 0.375):int(num_readings * 0.5)]  # 90° to 135°
            right = filtered_ranges[int(num_readings * 0.5):int(num_readings * 0.625)]  # 135° to 180°
            bottom_right = filtered_ranges[int(num_readings * 0.625):int(num_readings * 0.75)]  # 180° to 225°
            bottom = filtered_ranges[int(num_readings * 0.75):int(num_readings * 0.875)]  # 225° to 270°
            bottom_left = filtered_ranges[int(num_readings * 0.875):int(num_readings * 1.0)]  # 270° to 315°
            left = filtered_ranges[0:int(num_readings * 0.125)]  # 315° to 360°

            # Check each region for obstacles
            regions = {
                "Top Left": top_left,
                "Top": top,
                "Top Right": top_right,
                "Right": right,
                "Bottom Right": bottom_right,
                "Bottom": bottom,
                "Bottom Left": bottom_left,
                "Left": left,
            }
            
            for region_name, region_data in regions.items():
                if region_data:
                    min_range = min(region_data)  # Find the closest object in that region
                    if min_range < 1.0:
                        self.get_logger().warn(f"⚠️ Obstacle detected in {region_name}! Closest object at {min_range:.2f} meters.")
                    else:
                        self.get_logger().info(f"✅ Clear path . Closest object at {min_range:.2f} meters.")
                else:
                    self.get_logger().info(f"No valid range readings available in {region_name}.")
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

