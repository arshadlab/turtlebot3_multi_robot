from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # List of TurtleBot namespaces
    turtlebot_namespaces = ['tb1', 'tb2']  # Add more namespaces as needed

    # Create a list of Node actions for each TurtleBot
    nodes = []
    for namespace in turtlebot_namespaces:
        nodes.append(
            Node(
                package='monitor_position',
                executable='monitor_position',
                name=f'monitor_position_node_{namespace}',
                namespace=namespace,
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        )

    return LaunchDescription(nodes)