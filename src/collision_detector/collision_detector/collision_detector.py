#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from datetime import datetime

MIN_DISTANCE = 0.208 * 2 # Double the radius of the robot

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.subscription_tb1 = self.create_subscription(
            Odometry,
            'tb1/odom',
            self.position_callback_tb1,
            10
        )
        self.subscription_tb2 = self.create_subscription(
            Odometry,
            'tb2/odom',
            self.position_callback_tb2,
            10
        )
        self.collision_count = 0
        self.publisher = self.create_publisher(String, 'collision_alert', 10)
        
        self.tb1_position = None
        self.tb2_position = None

        self.timer = self.create_timer(0.1, self.check_collision)
        self.get_logger().info("Collision Detector Node has been started.")
    

    def position_callback_tb1(self, msg):
        self.tb1_position = msg.pose.pose.position
        
    def position_callback_tb2(self, msg):
        self.tb2_position = msg.pose.pose.position
        
        
    def check_collision(self):
        if self.tb1_position and self.tb2_position:
            distance = ((self.tb1_position.x - self.tb2_position.x) ** 2 + 
                        (self.tb1_position.y - self.tb2_position.y) ** 2 +
                        (self.tb1_position.z - self.tb2_position.z) ** 2) ** 0.5
            
            if distance < MIN_DISTANCE:
                current_time = self.get_clock().now().to_msg()
                self.get_logger().info(f"{current_time}| Collision detected between tb1 and tb2{distance:.2f}")
                msg = String()
                msg.data = f"{current_time}| Collision detected between tb1 and tb2{distance:.2f}"
                self.publisher.publish(msg)
                self.collision_count += 1

                
def main(args=None):
    rclpy.init(args=args)
    collision_detector = CollisionDetector()
    rclpy.spin(collision_detector)

    # Destroy the node explicitly
    collision_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  