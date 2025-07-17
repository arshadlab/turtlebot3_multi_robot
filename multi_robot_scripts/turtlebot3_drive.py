#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

DEG2RAD = math.pi / 180.0
LINEAR_VELOCITY = 0.15
ANGULAR_VELOCITY = 0.4

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3

CENTER = 0
LEFT = 1
RIGHT = 2

class Turtlebot3DriveStamped(Node):
    def __init__(self):
        super().__init__('turtlebot3_drive_node')

        self.scan_data = [0.0, 0.0, 0.0]
        self.robot_pose = 0.0
        self.prev_robot_pose = 0.0
        self.state = GET_TB3_DIRECTION
        #self.declare_parameter('use_sim_time', True)
        
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_timer(0.01, self.update_callback)
        self.get_logger().info("TurtleBot3 TwistStamped node initialized")

    def scan_callback(self, msg):
        angles = [0, 30, 330]
        for i, angle in enumerate(angles):
            self.scan_data[i] = msg.range_max if math.isinf(msg.ranges[angle]) else msg.ranges[angle]

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robot_pose = yaw

    def update_cmd_vel(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def update_callback(self):
        escape_range = 30.0 * DEG2RAD
        check_forward_dist = 0.7
        check_side_dist = 0.6

        if self.state == GET_TB3_DIRECTION:
            if self.scan_data[CENTER] > check_forward_dist:
                if self.scan_data[LEFT] < check_side_dist:
                    self.prev_robot_pose = self.robot_pose
                    self.state = TB3_RIGHT_TURN
                elif self.scan_data[RIGHT] < check_side_dist:
                    self.prev_robot_pose = self.robot_pose
                    self.state = TB3_LEFT_TURN
                else:
                    self.state = TB3_DRIVE_FORWARD
            elif self.scan_data[CENTER] < check_forward_dist:
                self.prev_robot_pose = self.robot_pose
                self.state = TB3_RIGHT_TURN

        elif self.state == TB3_DRIVE_FORWARD:
            self.update_cmd_vel(LINEAR_VELOCITY, 0.0)
            self.state = GET_TB3_DIRECTION

        elif self.state == TB3_RIGHT_TURN:
            if abs(self.prev_robot_pose - self.robot_pose) >= escape_range:
                self.state = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, -ANGULAR_VELOCITY)

        elif self.state == TB3_LEFT_TURN:
            if abs(self.prev_robot_pose - self.robot_pose) >= escape_range:
                self.state = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, ANGULAR_VELOCITY)

        else:
            self.state = GET_TB3_DIRECTION

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3DriveStamped()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
