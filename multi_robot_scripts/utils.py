import os
import yaml

"""
Loads a base Gazebo-ROS bridge YAML file and rewrites the topic names to include a robot-specific namespace.
Also updates message types from TwistStamped to Twist, if needed.
This utility adjusts TurtleBot3’s default bridge YAML config to support namespaces for ROS2 and gz.
The provided YAML file is reused but modified at runtime to prefix each topic with a namespace.
This is essential in multi-robot simulation, where each robot must publish and subscribe to its
own isolated set of topics (e.g., /tb0_1/cmd_vel) to avoid collisions.
It also normalizes message types (e.g., TwistStamped → Twist) where needed for compatibility.


Args:
    base_yaml_path (str): Path to the base YAML bridge configuration.
    namespace (str): Namespace prefix to apply to all ROS and Gazebo topic names.

Returns:
    str: Path to the modified, namespaced YAML file saved in /tmp.
"""

def create_namespaced_bridge_yaml(base_yaml_path, namespace):
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    namespaced_bridges = []

    for bridge in bridges:
        ros_topic = bridge['ros_topic_name']
        gz_topic = bridge['gz_topic_name']

        # Leave topics like /clock untouched (global topics)
        if ros_topic not in  ['clock']:
            bridge['ros_topic_name'] = f"{namespace}/{ros_topic}"
        if gz_topic not in  ['clock']:
            bridge['gz_topic_name'] = f"{namespace}/{gz_topic}"

        # Update type from TwistStamped to Twist
        if bridge['ros_type_name'] == "geometry_msgs/msg/TwistStamped":
            bridge['ros_type_name'] = "geometry_msgs/msg/Twist"
        if bridge['gz_type_name'] == "gz.msgs.TwistStamped":
            bridge['gz_type_name'] = "gz.msgs.Twist"

        namespaced_bridges.append(bridge)

    # Save to a temp or robot-specific YAML
    out_file = f"/tmp/{namespace}_bridge.yaml"
    with open(out_file, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return out_file

"""
This function loads an SDF model file and updates its topic definitions by prefixing them with a namespace.
Many simulation models (like TurtleBot3) have hardcoded topic names (e.g., <topic>cmd_vel</topic>).
To run multiple robots simultaneously, these topics must be isolated per robot using namespaces.  The gz plugins suppose
to prefix them with robot name but it's not. This function ensures that all relevant topic tags (for turtlebot3) in the SDF (like cmd_vel, odom, imu, etc.)
are updated accordingly so each instance operates independently in the simulation. 
For custom models, user must update below text accordingly.

Args:
    model_path (str): Path to the original SDF model file.
    namespace (str): Namespace to prepend to each topic.

Returns:
    str: Modified SDF content with namespaced topics.
"""
def load_sdf_with_namespace(model_path, namespace):
    with open(model_path, 'r') as f:
        sdf_text = f.read()

    # Define all plugin-related topic tags that should be updated
    topic_map = {
        '<tf_topic>/tf</tf_topic>':           f'<tf_topic>{namespace}/tf</tf_topic>',
        '<topic>cmd_vel</topic>':             f'<topic>{namespace}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>':      f'<odom_topic>{namespace}/odom</odom_topic>',
        '<topic>joint_states</topic>':        f'<topic>{namespace}/joint_states</topic>',
        '<topic>imu</topic>':                 f'<topic>{namespace}/imu</topic>',
        '<topic>scan</topic>':                f'<topic>{namespace}/scan</topic>',
        '<topic>camera/image_raw</topic>':    f'<topic>{namespace}/camera/image_raw</topic>',
        '<camera_info_topic>camera/camera_info</camera_info_topic>': f'<camera_info_topic>{namespace}/camera/camera_info</camera_info_topic>',
    }
    
    # Replace fixed topics with namespaced versions
    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    return sdf_text
