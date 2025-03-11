import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    turtlebot3_multi_robot_dir = get_package_share_directory('turtlebot3_multi_robot')
    monitor_position_dir = get_package_share_directory('monitor_position')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_drive = LaunchConfiguration('enable_drive', default='true')
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')

    # Include the Gazebo simulation launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_multi_robot_dir, 'launch', 'gazebo_multi_nav2_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_drive': enable_drive,
            'enable_rviz': enable_rviz
        }.items()
    )

    # Include the monitor position launch file
    monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(monitor_position_dir, 'launch', 'monitor_position_launch.py')
        )
    )

    return LaunchDescription([
        gazebo_launch,
        monitor_launch
    ])