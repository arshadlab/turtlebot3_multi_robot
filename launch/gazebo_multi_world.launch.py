#!/usr/bin/env python3

# This launch file is intended for educational and prototyping purposes.
# It demonstrates how to spawn multiple TurtleBot3 robots (Burger model) in Ignition Gazebo (Harmonic)
# using ROS 2 Jazzy. The robots are each given their own namespace and have individual bridges and state publishers.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from multi_robot_scripts.utils import load_sdf_with_namespace, create_namespaced_bridge_yaml

def launch_setup(context, *args, **kwargs):
    # Locate package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Robot configuration
    TURTLEBOT3_MODEL = "burger"
    model_folder = f"turtlebot3_{TURTLEBOT3_MODEL}"
    urdf_file_name = f"{model_folder}.urdf"

    # Read grid dimensions from launch args
    rows = int(LaunchConfiguration('rows').perform(context))
    cols = int(LaunchConfiguration('cols').perform(context))

    # Load URDF robot description
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Setup Gazebo world and model path
    world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_dqn_stage1.world')
    model_path = os.path.join(turtlebot3_gazebo_dir, 'models', model_folder, 'model.sdf')

    enable_drive = LaunchConfiguration("enable_drive", default="true").perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    # Add TurtleBot3 model path to Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models')
    )

    # Launch Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v4 {world_path}'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    last_action = None
    y = -cols

    # Spawn a grid of robots with namespaces and individual bridges
    for i in range(cols):
        x = -rows
        for j in range(rows):
            name = f"turtlebot{i}_{j}"
            namespace = f"/tb{i}_{j}"

            # Robot state publisher node
            state_pub = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                output="screen",
                parameters=[{
                    'use_sim_time': False,
                    'robot_description': robot_desc,
                    'frame_prefix': PythonExpression(["'", frame_prefix, "/'"]),
                    "publish_frequency": 10.0
                }],
                remappings=remappings,
            )

            # Load and patch SDF to apply namespace to topics
            sdf_string = load_sdf_with_namespace(model_path, namespace)

            # Gazebo entity spawn node
            spawn_robot = Node(
                package="ros_gz_sim",
                executable="create",
                namespace=namespace,
                arguments=[
                    "-string", sdf_string,
                    "-name", namespace,
                    "-x", str(x),
                    "-y", str(y),
                    "-z", "0.5",
                    "-Y", "3.14159",
                    "-unpause"
                ],
                output="screen",
            )

            # Dynamically adjust bridge YAML with namespace
            base_bridge_yaml = os.path.join(turtlebot3_gazebo_dir, 'params', f'{model_folder}_bridge.yaml')
            namespaced_yaml = create_namespaced_bridge_yaml(base_bridge_yaml, namespace)

            # Start topic bridge for each robot
            bridge_node = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=namespace,
                arguments=['--ros-args', '-p', f'config_file:={namespaced_yaml}'],
                output='screen'
            )

            # Start image bridge node (for camera streaming)
            image_bridge_node = Node(
                package='ros_gz_image',
                namespace=namespace,
                executable='image_bridge',
                arguments=['camera/image_raw'],
                output='screen'
            )

            # Ensure robots are spawned sequentially
            if last_action is None:
                ld.add_action(state_pub)
                ld.add_action(spawn_robot)
                ld.add_action(bridge_node)
                ld.add_action(image_bridge_node)
            else:
                ld.add_action(RegisterEventHandler(
                    OnProcessExit(
                        target_action=last_action,
                        on_exit=[state_pub, spawn_robot, bridge_node, image_bridge_node]
                    )
                ))

            last_action = spawn_robot
            x += 1.0
        y += 1.0

    # Launch drive node for each robot after they are all spawned
    for i in range(cols):
        for j in range(rows):
            namespace = f"/tb{i}_{j}"
            drive_node = Node(
                package="turtlebot3_gazebo",
                executable="turtlebot3_drive",
                namespace=namespace,
                output="screen",
                condition=IfCondition(enable_drive),
            )
            drive_event = RegisterEventHandler(
                OnProcessExit(
                    target_action=last_action,
                    on_exit=[drive_node],
                )
            )
            ld.add_action(drive_event)

    return [ld]

def generate_launch_description():
    # Declare launch arguments for rows, cols, and drive control
    return LaunchDescription([
        DeclareLaunchArgument("rows", default_value="2"),
        DeclareLaunchArgument("cols", default_value="2"),
        DeclareLaunchArgument("enable_drive", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])
