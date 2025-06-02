import rclpy
from rclpy.node import Node
import rtamt
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from threading import Lock
import re
import math
import traceback
from time import sleep
from datetime import datetime


epsilon = 1e-5

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        self.cur_namespace = self.get_namespace().strip('/')

        self.all_satisfied = True

        self.future_horizont = 5  # Number of time steps to keep in history (window)
        self.collision_threshold =  0.208 * 2  # meters
        self.delta_t = 0.2  # prediction interval

        # Use RTAMT Discrete Time OFFLINE Monitor
        self.monitor = rtamt.StlDiscreteTimeSpecification()
        self.monitor.name = "CollisionMonitor"

        # Own position and velocity
        self.monitor.declare_var("x1_f", "float")
        self.monitor.declare_var("y1_f", "float")
        # Other robot position and velocity
        self.monitor.declare_var("x2_f", "float")
        self.monitor.declare_var("y2_f", "float")

        self.monitor.spec = f"""
        G[0,{self.future_horizont-1}] (
            pow(x1_f - x2_f, 2) + pow(y1_f - y2_f, 2) > pow({self.collision_threshold}, 2)
        )
        """
        self.monitor.parse()

        # History buffer for offline monitor (just values, not timestamps)
        self.history = {'x1_f': [], 'y1_f': [], 'x2_f': [], 'y2_f': []}

        # Section only valid for the 2 robot case
        match = re.match(r'(tb)(\d+)', self.cur_namespace)
        if match:
            letters, numbers = match.groups()
            numbers = (int(numbers) % 2) + 1
            self.ext_namespace = f'{letters}{numbers}'

        # Subscribe to ROS2 odometry topics
        self.own_subscription = self.create_subscription(
            Odometry,
            f'/{self.cur_namespace}/odom',
            self.listener_callback1,
            10
        )
        self.ext_subscription = self.create_subscription(
            Odometry,
            f'/{self.ext_namespace}/odom',
            self.listener_callback2,
            10
        )
        self.timer = self.create_timer(0.1, self.collision_callback)
        self.publisher = self.create_publisher(String, 'collision_warning', 10)
        self.mutex = Lock()
        self.position1 = {}
        self.position2 = {}

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat
        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)  # clamp
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def collision_callback(self):
        with self.mutex:
            # Wait for the two robots to publish their positions
            if not self.position1 or not self.position2:
                return

            x1_future = self.position1['x1'] + self.position1['v1'] * self.delta_t * math.cos(self.position1['theta1'])
            y1_future = self.position1['y1'] + self.position1['v1'] * self.delta_t * math.sin(self.position1['theta1'])
            x2_future = self.position2['x2'] + self.position2['v2'] * self.delta_t * math.cos(self.position2['theta2'])
            y2_future = self.position2['y2'] + self.position2['v2'] * self.delta_t * math.sin(self.position2['theta2'])

            # Add new values to history (no timestamps, just the predicted value)
            self.history['x1_f'].append(x1_future)
            self.history['y1_f'].append(y1_future)
            self.history['x2_f'].append(x2_future)
            self.history['y2_f'].append(y2_future)

            # Keep only the last N values
            max_len = self.future_horizont
            for key in self.history:
                if len(self.history[key]) > max_len:
                    self.history[key] = self.history[key][-max_len:]

            n= len(self.history['x1_f'])
            if n < 2:
                self.get_logger().info("Not enough data points to evaluate STL formula.")
                return
            dataset = {
                'time': list(range(n)),
                'x1_f': self.history['x1_f'],
                'y1_f': self.history['y1_f'],
                'x2_f': self.history['x2_f'],
                'y2_f': self.history['y2_f'],
            }

            try:
                robustness = self.monitor.evaluate(dataset)
                if isinstance(robustness, list) and len(robustness) > 0:
                    satisfied = all(r[1] > 0 for r in robustness)
                else:
                    satisfied = robustness > epsilon
                if not satisfied:
                    dx = self.position1['x1'] - self.position2['x2']
                    dy = self.position1['y1'] - self.position2['y2']
                    distance_now = math.sqrt(dx**2 + dy**2)
                    current_time = self.get_clock().now().to_msg()
                    self.get_logger().warn(f"{current_time}Collision detected or too close! Distance now: {distance_now:.2f} m")
                    
            except Exception as e:
                self.get_logger().error(f"STL evaluation failed: {e} (type: {type(e)})")
                self.get_logger().error("Traceback:\n" + traceback.format_exc())
                return

    def listener_callback1(self, msg):
        with self.mutex:
            try:
                x_position = msg.pose.pose.position.x
                y_position = msg.pose.pose.position.y
                velocity = msg.twist.twist.linear
                orientation = msg.pose.pose.orientation
                orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                _, _, yaw = self.euler_from_quaternion(orientation_list)
                theta = yaw
                overall_velocity = math.sqrt(velocity.x**2 + velocity.y**2)
                self.position1 = {
                    'x1': x_position,
                    'y1': y_position,
                    'v1': overall_velocity,
                    'theta1': theta
                }
            except Exception as e:
                self.get_logger().error(f"listener_callback1 error: {e}")

    def listener_callback2(self, msg):
        with self.mutex:
            try:
                x_position = msg.pose.pose.position.x
                y_position = msg.pose.pose.position.y
                velocity = msg.twist.twist.linear
                orientation = msg.pose.pose.orientation
                orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
                _, _, yaw = self.euler_from_quaternion(orientation_list)
                theta = yaw
                overall_velocity = math.sqrt(velocity.x**2 + velocity.y**2)
                self.position2 = {
                    'x2': x_position,
                    'y2': y_position,
                    'v2': overall_velocity,
                    'theta2': theta
                }
            except Exception as e:
                self.get_logger().error(f"listener_callback2 error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
