from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_waypoint_follower',
            executable='twist_waypoint_follower',
            name='twist_waypoint_follower',
            namespace='',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
