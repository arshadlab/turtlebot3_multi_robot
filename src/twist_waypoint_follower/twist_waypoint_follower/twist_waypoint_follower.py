import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

WAYPOINTS = [
    (-0.5, 1.5),
    (-0.5, -1.5),
    (-1.5, -1.5),
    (-1.5, 1.5)
]

class TwistWaypointFollower(Node):
    def __init__(self):
        super().__init__('twist_waypoint_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(
            Odometry, '/tb1/odom', self.odom_callback, 10)
        self.current_pose = None
        self.current_waypoint = 0
        self.get_logger().info("Twist Waypoint Follower Node has been started.")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def timer_callback(self):
        if self.current_pose is None:
            return
        self.get_logger().info(f"Current Pose: {self.current_pose}")
        self.get_logger().info(f"Current Waypoint: {self.current_waypoint}")
        target = WAYPOINTS[self.current_waypoint]
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        angle_diff = self.normalize_angle(angle_to_target - yaw)

        msg = Twist()
        # First rotate toward the target
        if abs(angle_diff) > 0.1:
            msg.angular.z = 0.5 * angle_diff
        # Then move forward when facing the target
        elif distance > 0.05:
            msg.linear.x = 0.3
        else:
            # Switch to next waypoint
            self.current_waypoint = (self.current_waypoint + 1) % len(WAYPOINTS)
        self.publisher_.publish(msg)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TwistWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
