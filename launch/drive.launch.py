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

import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    tb3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    robot_config_file = os.path.join(
        get_package_share_directory('tb3_multi_robot'),
        'config',
        'robots.yaml'
        )

    with open(robot_config_file, 'r') as f:
        config = yaml.safe_load(f)

    robots = [r for r in config['robots'] if r.get('enabled', True)]
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    for robot in robots:
        namespace = f"{robot['name']}"
        namespace_str = '/' + namespace
        drive_node = Node(
                package="tb3_multi_robot",
                executable="turtlebot3_drive",
                namespace=namespace_str,
                parameters=[{
                    'use_sim_time': use_sim_time}],
                output="screen",
            )

        ld.add_action(drive_node)


    return ld