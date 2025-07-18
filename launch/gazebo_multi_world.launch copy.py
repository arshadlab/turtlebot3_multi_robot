#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Joep Tool, ChanHyeong Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from multi_robot_scripts.utils import load_sdf_with_namespace, create_namespaced_bridge_yaml

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )
    
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_dqn_stage1.world'
    )
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'
        )
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    ROWS = 2
    COLS = 2

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    TURTLEBOT3_MODEL = "burger"
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', f'turtlebot3_{TURTLEBOT3_MODEL}', 'model.sdf'
    )
 
    # Spawn turtlebot3 instances in gazebo
    for i in range(COLS):
        x = -ROWS
        for j in range(ROWS):
            # Construct a unique name and namespace
            name = "turtlebot" + str(i) + "_" + str(j)
            namespace = "/tb" + str(i) + "_" + str(j)
            # Create state publisher node for that instance

            turtlebot_state_publisher = Node(
                package="robot_state_publisher",
                namespace=namespace,
                executable="robot_state_publisher",
                output="screen",
                parameters=[{
                'use_sim_time': False,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"]),
                "publish_frequency": 10.0
                }],
                remappings=remappings,
            )
            
            # Load and patch the SDF content
            sdf_string = load_sdf_with_namespace(model_path, namespace)
            #print(f"Spawning {sdf_string}")
            # Create spawn call
            spawn_turtlebot3 = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=namespace,
                arguments=[
                    '-string', sdf_string,
                    "-name", namespace,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.5",
                    "-Y", "3.14159",
                    "-unpause",
                ],
                output="screen",
            )
            
            x += 3.0
            
            base_bridge_file = os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'params',
                model_folder+'_bridge.yaml'
            )

            # Dynamically patch topic names
            bridge_params = create_namespaced_bridge_yaml(base_bridge_file, namespace)

            start_gazebo_ros_bridge_cmd = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                arguments=[
                    '--ros-args',
                    '-p',
                    f'config_file:={bridge_params}',
                ],
                output='screen',
            )

            start_gazebo_ros_image_bridge_cmd = Node(
                package='ros_gz_image',
                namespace=namespace,
                executable='image_bridge',
                arguments=['camera/image_raw'],
                output='screen',
            )
            
           
            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(turtlebot_state_publisher)
                ld.add_action(spawn_turtlebot3)
                ld.add_action(start_gazebo_ros_bridge_cmd)
                ld.add_action(start_gazebo_ros_image_bridge_cmd)
                
            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_turtlebot3_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[spawn_turtlebot3,
                                 turtlebot_state_publisher,
                                 start_gazebo_ros_bridge_cmd,
                                 start_gazebo_ros_image_bridge_cmd
                                 ],
                    )
                )
                ld.add_action(spawn_turtlebot3_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_turtlebot3

        y += 3.0
    
         # Start all driving nodes after the last robot is spawned
    for i in range(COLS):
        for j in range(ROWS):
            namespace = "/tb" + str(i) + "_" + str(j)
            # Create spawn call
            drive_turtlebot3_burger = Node(
                package="turtlebot3_gazebo",
                executable="turtlebot3_drive",
                namespace=namespace,
                output="screen",
                condition=IfCondition(enable_drive),
            )

            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            drive_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[drive_turtlebot3_burger],
                )
            )
            
            ld.add_action(drive_turtlebot3_event)
    
    return ld
