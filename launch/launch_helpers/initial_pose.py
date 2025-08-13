#!/bin/python3

import math
import argparse
import sys
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseWithCovarianceStamped

def yaw_to_quat_z(yaw: float):
    # planar Z-only quaternion
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class InitialPosePublisher(Node):
    def __init__(self, ns: str, x: float, y: float, yaw: float,
                 std_xy: float, std_yaw: float,
                 frame_id: str, repeat: int, rate_hz: float):
        super().__init__('initialpose_publisher', namespace=ns)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = frame_id

        qx, qy, qz, qw = yaw_to_quat_z(yaw)
        self.msg.pose.pose.position.x = x
        self.msg.pose.pose.position.y = y
        self.msg.pose.pose.position.z = 0.0
        self.msg.pose.pose.orientation.x = qx
        self.msg.pose.pose.orientation.y = qy
        self.msg.pose.pose.orientation.z = qz
        self.msg.pose.pose.orientation.w = qw

        # 6x6 covariance (row-major). Set x, y, and yaw (theta) variances.
        cov = [0.0] * 36
        var_xy = std_xy * std_xy
        var_yaw = std_yaw * std_yaw
        cov[0] = var_xy          # x
        cov[7] = var_xy          # y
        cov[35] = var_yaw        # yaw (theta about Z)
        self.msg.pose.covariance = cov

        self.left = max(1, repeat)
        period = 1.0 / max(0.1, rate_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"Publishing {self.left} initialpose message(s) to '{self.get_namespace()}/initialpose' "
            f"(x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad, frame_id='{frame_id}')"
        )

    def _tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.left -= 1
        if self.left <= 0:
            self.get_logger().info("Done publishing initial pose.")
            rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ns', default='', help="Robot namespace (e.g., tb1, tb2). No leading slash needed.")
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    # Provide yaw in radians or degrees (deg wins if provided)
    parser.add_argument('--yaw', type=float, default=None, help="Yaw in radians.")
    parser.add_argument('--yaw-deg', type=float, default=None, help="Yaw in degrees (overrides --yaw if set).")
    parser.add_argument('--std-xy', type=float, default=0.25, help="Std dev for x/y (meters).")
    parser.add_argument('--std-yaw', type=float, default=math.radians(15.0), help="Std dev for yaw (radians).")
    parser.add_argument('--frame-id', default='map', help="Frame id for the pose (default: map).")
    parser.add_argument('--repeat', type=int, default=5, help="Times to publish (default: 5).")
    parser.add_argument('--rate', type=float, default=2.0, help="Publish rate Hz (default: 2).")
    clean_argv = remove_ros_args(args = sys.argv)
    args = parser.parse_args(clean_argv[1:])

    yaw = math.radians(args.yaw_deg) if args.yaw_deg is not None else (args.yaw if args.yaw is not None else 0.0)

    rclpy.init()
    node = InitialPosePublisher(
        ns=args.ns.strip('/'),
        x=args.x, y=args.y, yaw=yaw,
        std_xy=args.std_xy, std_yaw=args.std_yaw,
        frame_id=args.frame_id,
        repeat=args.repeat, rate_hz=args.rate
    )
    rclpy.spin(node)

if __name__ == '__main__':
    main()
