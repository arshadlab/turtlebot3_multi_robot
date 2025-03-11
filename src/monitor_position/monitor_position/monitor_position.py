#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rtamt
from nav_msgs.msg import Odometry

class ZPositionMonitor(Node):
    def __init__(self):
        super().__init__('z_position_monitor')

        # Create RTAMT monitor
        self.monitor = rtamt.STLDiscreteTimeOnlineSpecification()
        self.monitor.name = "Z-Position Monitor"
        self.monitor.declare_var("z", "float")
        self.monitor.set_var_io_type("z", "input")
        self.monitor.spec = "G(z >= 0 and z <= 10)"  # Z must be within [0, 10]
        self.monitor.parse()

        # Subscribe to ROS2 topic (e.g., /odom)
        # Decission to be made: individual topics or a single topic with multiple namespaces
        self.subscription = self.create_subscription(
            Odometry,
            'odom', 
            self.listener_callback,
            10
        )
        self.last_timestamp = None

    def listener_callback(self, msg):
        # Extract z-position from Odometry message
        z_position = msg.pose.pose.position.z

        # Use current time as timestamp (in seconds)
        timestamp = self.get_clock().now().nanoseconds // 1e9

        # Evaluate STL specification
        robustness = self.monitor.update(timestamp, {"z": z_position})
        is_satisfied = robustness > 0

        # Print results
        self.get_logger().info(f"Time: {timestamp}, Z-Position: {z_position}, Robustness: {robustness}, Satisfied: {is_satisfied}")

def main(args=None):
    rclpy.init(args=args)
    node = ZPositionMonitor()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




