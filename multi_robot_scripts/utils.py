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
    """Create a temporary namespaced bridge YAML for ros_gz_bridge."""
    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    if namespace and not namespace.endswith('/'):
        namespace_with_slash = namespace + '/'
    else:
        namespace_with_slash = namespace

    namespaced_bridges = []
    for bridge in bridges:
        if bridge['ros_topic_name'] not in ['clock']:
            bridge['ros_topic_name'] = f"{namespace_with_slash}{bridge['ros_topic_name']}"
        if bridge['gz_topic_name'] not in ['clock']:
            bridge['gz_topic_name'] = f"{namespace_with_slash}{bridge['gz_topic_name']}"
        namespaced_bridges.append(bridge)

    output_path = f"/tmp/{namespace.strip('/')}_bridge.yaml"
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path

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
    """Patch SDF file to inject robot namespace into all relevant topic tags."""
    with open(model_path, 'r') as f:
        sdf_text = f.read()

    topic_map = {
        '<tf_topic>/tf</tf_topic>': f'<tf_topic>{namespace}/tf</tf_topic>',
        '<topic>cmd_vel</topic>': f'<topic>{namespace}/cmd_vel</topic>',
        '<odom_topic>odom</odom_topic>': f'<odom_topic>{namespace}/odom</odom_topic>',
        '<topic>joint_states</topic>': f'<topic>{namespace}/joint_states</topic>',
        '<topic>imu</topic>': f'<topic>{namespace}/imu</topic>',
        '<topic>scan</topic>': f'<topic>{namespace}/scan</topic>',
        '<topic>camera/image_raw</topic>': f'<topic>{namespace}/camera/image_raw</topic>',
        '<camera_info_topic>camera/camera_info</camera_info_topic>': f'<camera_info_topic>{namespace}/camera/camera_info</camera_info_topic>',
    }

    for original, replacement in topic_map.items():
        sdf_text = sdf_text.replace(original, replacement)

    return sdf_text

"""
This function generates a namespaced RViz configuration file for a TurtleBot3 robot.
"""
def generate_rviz_config(robot_name, base_config_path):
    # Read the base RViz config
    with open(base_config_path, 'r') as f:
        config = f.read()

    # Replace placeholders
    config = config.replace('<ROBOT_NAME>', robot_name)

    # Use system temp directory
    temp_dir = tempfile.gettempdir()
    output_config_path = os.path.join(temp_dir, f'{robot_name}_rviz_config.rviz')

    with open(output_config_path, 'w') as f:
        f.write(config)

    return output_config_path
