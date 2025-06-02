import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

TB1_WAYPOINTS = [
    (-1.5, -1.5),
    (-0.5, -1.5),
    (-0.5, 1.5),
    (-1.5, 1.5)
]


TB2_WAYPOINTS = [
    (0.5, 0.5),
    (0.5,1.5),
    (-0.5,1.5),
    (-0.5,-1.5),
    (0.5,-1.5),
    (0.5,-0.5),
    (1.5,-0.5),
    (1.5,0.5)
]

class TwistWaypointFollower(Node):
    def __init__(self):
        super().__init__('twist_waypoint_follower')
        self.publisher_tb1 = self.create_publisher(Twist, '/tb1/cmd_vel', 10)
        self.subscriber_tb1 = self.create_subscription(
            Odometry, '/tb1/odom', self.odom_callback_tb1, 10)
        
        self.publisher_tb2 = self.create_publisher(Twist, '/tb2/cmd_vel', 10)
        self.subscriber_tb2 = self.create_subscription(
            Odometry, '/tb2/odom', self.odom_callback_tb2, 10)
        self.current_pose_tb1 = None
        self.current_pose_tb2 = None
        self.current_waypoint_tb1 = 0
        self.current_waypoint_tb2 = 0
        self.get_logger().info("Twist Waypoint Follower Node has been started.")
        self.timer1 = self.create_timer(0.1, self.timer_callback_tb1)
        self.timer2 = self.create_timer(0.1, self.timer_callback_tb2)

    def odom_callback_tb1(self, msg):
        self.current_pose_tb1 = msg.pose.pose
    def odom_callback_tb2(self,msg):
        self.current_pose_tb2 = msg.pose.pose
    def timer_callback_tb1(self):
        if self.current_pose_tb1 is None:
            return
        target = TB1_WAYPOINTS[self.current_waypoint_tb1]
        x = self.current_pose_tb1.position.x
        y = self.current_pose_tb1.position.y
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        yaw = self.quaternion_to_yaw(self.current_pose_tb1.orientation)
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
            self.current_waypoint_tb1 = (self.current_waypoint_tb1 + 1) % len(TB1_WAYPOINTS)
        self.publisher_tb1.publish(msg)


    def timer_callback_tb2(self):
        if self.current_pose_tb2 is None:
            return

        target = TB2_WAYPOINTS[self.current_waypoint_tb2]
        x = self.current_pose_tb2.position.x
        y = self.current_pose_tb2.position.y
        dx = target[0] - x
        dy = target[1] - y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        yaw = self.quaternion_to_yaw(self.current_pose_tb2.orientation)
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
            self.current_waypoint_tb2 = (self.current_waypoint_tb2 + 1) % len(TB2_WAYPOINTS)
        self.publisher_tb2.publish(msg)

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
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
