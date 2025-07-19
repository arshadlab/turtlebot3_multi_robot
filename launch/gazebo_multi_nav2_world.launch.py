#!/usr/bin/env python3

# This launch file is intended for educational and prototyping purposes.
# It demonstrates how to spawn multiple TurtleBot3 robots (e.g Burger model) in Ignition Gazebo (Harmonic)
# using ROS 2 Jazzy. The robots are each given their own namespace and have individual bridges and state publishers.


#if using cyclone dds
#export CYCLONEDDS_URI=${CYCLONEDDS_URI:-"<CycloneDDS><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></CycloneDDS>"}


#export GZ_FUEL_CACHE_ONLY=1

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from multi_robot_scripts.utils import load_sdf_with_namespace, create_namespaced_bridge_yaml

def launch_setup(context, *args, **kwargs):
    
     # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        #{'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        #{'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        #{'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]
    
    enable_drive = LaunchConfiguration("enable_drive", default="true").perform(context)
    frame_prefix = LaunchConfiguration('frame_prefix', default='')

    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )
    # Robot configuration
    TURTLEBOT3_MODEL = "burger"
    model_folder = f"turtlebot3_{TURTLEBOT3_MODEL}"
    urdf_file_name = f"{model_folder}.urdf"
    
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )
    
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # Load URDF robot description
    urdf_path = os.path.join(turtlebot3_gazebo_dir, 'urdf', urdf_file_name)
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Locate package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

 

    # Setup Gazebo world and model path
    #world_path = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_dqn_stage1.world')
    world_path = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        'worlds', 'multi_robot_world.world')
    model_path = os.path.join(turtlebot3_multi_robot, 'models', model_folder, 'model.sdf')

   
    # Add TurtleBot3 model path to Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models')
    )

    # Add this at the top level of your generate_launch_description or launch_setup function
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    set_gz_sim_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=turtlebot3_gazebo_dir + '/models'
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


    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    
    ld = LaunchDescription()
    ld.add_action(set_gz_sim_resource)
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_env_vars_resources)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])
    
    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    
    last_action = None
 
    # Spawn a grid of robots with namespaces and individual bridges
    for robot in robots:
        namespace = f"{robot['name']}"
        # Robot state publisher node
        state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': PythonExpression(["'", frame_prefix, "/'"]),
                "publish_frequency": 10.0
            }],
            remappings=remappings,
        )

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
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
                "-x", robot['x_pose'],
                "-y", robot['y_pose'],
                "-z", "0.5",
                "-Y", "3.14159",
                "-unpause"
            ],
            output="screen",
        )

        # Dynamically adjust bridge YAML with namespace
        base_bridge_yaml = os.path.join(turtlebot3_multi_robot, 'params', f'{model_folder}_bridge.yaml')
        namespaced_yaml = create_namespaced_bridge_yaml(base_bridge_yaml, namespace)

        # Start topic bridge for each robot
        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            #namespace=namespace,
            arguments=['--ros-args', '-p', f'config_file:={namespaced_yaml}'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
        '''
        # Start image bridge node (for camera streaming)
        image_bridge_node = Node(
            package='ros_gz_image',
            namespace=namespace,
            executable='image_bridge',
            arguments=['camera/image_raw'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
        '''
        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )
        
        # Ensure robots are spawned sequentially
        if last_action is None:
            ld.add_action(state_pub)
            ld.add_action(spawn_robot)

            ld.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[bridge_node, bringup_cmd]
                )
            ))
            
        else:
            ld.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action=last_action,
                    on_exit=[state_pub, spawn_robot]
                )
            ))
            
            ld.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[bridge_node, bringup_cmd]
                )
            ))

        last_action = spawn_robot
        
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    ld.add_action(RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[clock_bridge_node]
                )
            ))
    
    for robot in robots:

        namespace = f"/{robot['name']}"

        # Create a initial pose topic publish call
        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', [namespace] + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
             parameters=[{'use_sim_time': use_sim_time}],
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)
        ld.add_action(declare_params_file_cmd)
    
    return [ld]

def generate_launch_description():
    # Declare launch arguments for rows, cols, and drive control
    return LaunchDescription([
        DeclareLaunchArgument("rows", default_value="2"),
        DeclareLaunchArgument("cols", default_value="2"),
        DeclareLaunchArgument("enable_drive", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])
