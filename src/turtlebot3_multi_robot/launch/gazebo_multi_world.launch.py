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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = "burger"

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )


    turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot")
    launch_file_dir = os.path.join(turtlebot3_multi_robot, "launch")

    world = os.path.join(
        turtlebot3_multi_robot, "worlds", "multi_empty_world.world"
    )

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        turtlebot3_multi_robot, "urdf", urdf_file_name
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ROWS = 5
    COLS = 5

    x = -ROWS
    y = -COLS
    last_action = None

    # Remapping is required for state publisher otherwise /tf and /tf_static will get be published on root '/' namespace
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

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
                parameters=[{"use_sim_time": False,
                             "publish_frequency": 10.0}],
                remappings=remappings,
                arguments=[urdf],
            )

            # Create spawn call
            spawn_turtlebot3_burger = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                    "-entity",
                    name,
                    "-robot_namespace",
                    namespace,
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    "0.01",
                    "-Y",
                    "3.14159",
                    "-unpause",
                ],
                output="screen",
            )

            # Advance by 2 meter in x direction for next robot instantiation
            x += 2.0

            if last_action is None:
                # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
                ld.add_action(turtlebot_state_publisher)
                ld.add_action(spawn_turtlebot3_burger)
                
            else:
                # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
                # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
                spawn_turtlebot3_event = RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=last_action,
                        on_exit=[spawn_turtlebot3_burger,
                                 turtlebot_state_publisher],
                    )
                )
                ld.add_action(spawn_turtlebot3_event)

            # Save last instance for next RegisterEventHandler
            last_action = spawn_turtlebot3_burger

        # Advance by 2 meter in y direction for next robot instantiation
        y += 2.0

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
