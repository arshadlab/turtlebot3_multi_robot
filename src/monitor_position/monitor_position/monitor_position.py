#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rtamt
from nav_msgs.msg import Odometry




class ZPositionMonitor(Node):
    def __init__(self):
        super().__init__('z_position_monitor')
        self.all_satisfied = True

        # Initialize RTAMT dense-time online monitor
        self.monitor = rtamt.StlDenseTimeOnlineSpecification()
        self.monitor.name = "Z-Position Monitor"
        self.monitor.declare_var("z", "float")
        self.monitor.set_var_io_type("z", "input")
        self.monitor.declare_var("is_correct", "int")
        self.monitor.set_var_io_type("is_correct", "output")
        self.monitor.spec = "z >= 0 and z <= 10"  # Ensure z is within [0, 10]
        print(f"Monitor: {self.monitor.spec}")
        self.monitor.set_sampling_period(0.2)
        self.monitor.parse()
        self.monitor.update(('z',[(0.0, 0.0)]))  # Initialize monitor

        # Subscribe to ROS2 odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        epsilon = 1e-9
        try:
            # Extract z-position from Odometry message
            z_position = msg.pose.pose.position.z
            x_position = msg.pose.pose.position.x
            y_position = msg.pose.pose.position.y
            self.get_logger().info(f"X-Position: {x_position:.3f}, Y-Position: {y_position:.3f}, Z-Position: {z_position:.3f}\nZ-Position Type: {type(z_position)}")
            # Check if z_position is a float
            # Get current ROS2 time in seconds (dense time)
            timestamp = self.get_clock().now().nanoseconds / 1e9

            # Correct dataset format: dictionary with list of (time, value) tuples
            dataset = [(timestamp, z_position)]

            # Evaluate STL specification using RTAMT dense-time online monitor
            robustness = self.monitor.update(('z', dataset))
            if robustness is None:
                self.get_logger().info("Robustness is None")
                return
            is_satisfied = True
            if len(robustness) > 0:
                is_satisfied = robustness[0][1] > epsilon  
            
            if not is_satisfied:
                self.all_satisfied = False  # If there's a single failure, it stays false
        except KeyError as e:
            self.get_logger().error(f"KeyError encountered: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

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
