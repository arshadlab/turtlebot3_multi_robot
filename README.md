
# Multi-TurtleBot3 Simulation with ROS 2 Jazzy & Gazebo Harmonic
This repository provides a scalable ROS 2-based framework to simulate multiple TurtleBot3 robots in Gazebo with Navigation2 (Nav2) support. Each robot runs within its own namespace, enabling clean separation and interaction-free operation.

The 'master' branch is updated with Jazzy support.  The 'humble' branch includes an implementation that functions with the humble framework, while the 'foxy' branch provides support specifically for ROS2 Foxy.

## Branch Mapping
'master' -> ROS2 Jazzy

'humble' -> ROS2 Humble

'foxy' -> ROS2 Foxy

The Jazzy version features a streamlined multi-robot setup that improves usability and launch flexibility.

## Prerequisites

- **Operating System**: Ubuntu 24.04
- **ROS Version**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- **Gazebo Version**: Gazebo Harmonic

Refer to the official ROS2 Jazzy installation guide: [link](https://docs.ros.org/en/jazzy/Installation.html)

### Install Required Dependencies

```
apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    curl \
    ros-jazzy-rmw-implementation \
    ros-jazzy-rmw-cyclonedds-cpp
```

When launching multiple robots with Nav2, the number of DDS participants can quickly exceed the default limit set by CycloneDDS. To avoid participant ID exhaustion, create a configuration file to increase the allowable range:

Create a file with below contents
e.g `$HOME/cyclonedds.xml`
```
<CycloneDDS>
  <Discovery>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
  </Discovery>
</CycloneDDS>
```

Set the CYCLONEDDS_URI environment variable to point to the XML configuration file. To make this persistent across terminal sessions, add the export command to ~/.bashrc:

```
export CYCLONEDDS_URI=$HOME/cyclonedds.xml
```

## Setup Workspace and Clone Repository

```
$ mkdir -p robot_ws/src
$ cd robot_ws/src

# Clone the Jazzy branch of the multi-robot repo
$ git clone  https://github.com/arshadlab/tb3_multi_robot.git -b jazzy

# Initialize the workspace
$ cd robot_ws
$ source /opt/ros/jazzy/setup.bash
$ rosdep install --from-paths src -r -y
```

 It's recommended to download the default model assets to ensure proper rendering and simulation behavior.

```
$ mkdir -p ~/.gazebo/models
$ git clone https://github.com/osrf/gazebo_models ~/.gazebo/models
```

## 🔧 Build the Workspace
After installing dependencies and setting up the workspace, compile the ROS 2 packages using colcon:

```
$ cd robot_ws/
$ colcon build --symlink-install
$ source ./install/setup.bash
```

Then, update the config/robots.yaml file to define the robot setup.
By default, four robots (tb1, tb2, tb3, tb4) are listed, with only tb1 and tb3 enabled. Modify the names, positions, and enabled flags as needed.

## 🚀 Launch the Simulation (Robots Only)
Use the following command to start the Gazebo simulation with the configured TurtleBot3 robots:

```
$ ros2 launch tb3_multi_robot tb3_world.launch.py
```
<img width="1840" height="1004" alt="image" src="https://github.com/user-attachments/assets/68d08e6a-8ab6-4f3d-98b3-504102b96312" />

After the simulation is launched, the system can either proceed with the Nav2 stack for autonomous navigation or use driving nodes for manual or scripted control.
A Python-based turtlebot3_drive script is included, replicating the original C++ node functionality while addressing compatibility issues with ROS 2 Jazzy and Gazebo Harmonic.

## 🚗 Launch Driving Nodes (Optional)
The original turtlebot3_drive application is not fully compatible with ROS 2 Jazzy and Gazebo Harmonic due to message type differences (e.g., use of TwistStamped for the /cmd_vel topic).
To address this, a Python-based equivalent is provided using compatible message types.

To launch the driving node for each robot, use the command below.
While a drive.launch.py file is included for automated multi-robot support, it is still under development and may require manual execution for each robot.

```
$ ./install/tb3_multi_robot/bin/turtlebot3_drive --ros-args -r __ns:=/tb1
```

Replace /tb1 with the appropriate robot namespace (/tb2, /tb3, etc.) as defined in robot configuration.

## 🧭 Launch Navigation2 Stack

With the robots running in Gazebo (via tb3_world.launch.py), the Navigation2 (Nav2) stack can be launched from a separate terminal.

```
$ ros2 launch  tb3_multi_robot tb3_nav2.launch.py
```
This will launch Nav2 nodes for all enabled robots using their respective namespaces.

<img width="2172" height="1721" alt="image" src="https://github.com/user-attachments/assets/d4c9e2ff-9721-4711-8e89-c25acbb3b207" />

The RViz2 panel title has been updated to include the corresponding robot name, making it easier to match each RViz instance with its respective robot in the Gazebo simulation.

### 🧭 Set Initial Pose
With RViz running, the robot’s initial position and orientation can be set using the 2D Pose Estimate button, aligned to its simulated placement and heading.

<img width="2180" height="1676" alt="image" src="https://github.com/user-attachments/assets/6a0c1bb0-32b1-4cf2-ad74-2da1ffba88e7" />

Alternatively, the initial pose can be set programmatically via the command line when the robot’s position and orientation are known from the simulation.

To retrieve the live pose of robots from Gazebo Harmonic, run:

```
$ gz topic -e -t /world/default/pose/info
```
This command lists the poses of all simulated entities. Identify the target robot by its name, such as tb1_waffle or tb1_burger.

Update the sample commands below with the position and orientation values obtained. The covariance typically remains unchanged.
Below commands are given for included robots.  User will need to update them for any custom robots. 

```
# TB1
$ ros2 topic pub --once /tb1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: -0.5000000060919606, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
               
# TB2
$ ros2 topic pub --once /tb2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: -1.500653720729433, y: 0.49999999390803895, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
   
# TB3
$ ros2 topic pub --once /tb3/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: -0.50000000609196049, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
  
# TB4
$ ros2 topic pub --once /tb4/initialpose geometry_msgs/msg/PoseWithCovarianceStamped "header:
  frame_id: 'map'
pose:
  pose:
    position: {x: 1.499346279270567, y: 0.499999993908039, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -1.9562638e-06, w: 0.99999595}
  covariance: [0.25, 0, 0, 0, 0, 0,
               0, 0.25, 0, 0, 0, 0,
               0, 0, 0.0001, 0, 0, 0,
               0, 0, 0, 0.0001, 0, 0,
               0, 0, 0, 0, 0.0001, 0,
               0, 0, 0, 0, 0, 0.06853892]"
```

### 🎯 Set Navigation Goal

A navigation goal may be sent using the 'Nav2 Goal' button in RViz after launching the Nav2 stack.

<img width="1091" height="842" alt="image" src="https://github.com/user-attachments/assets/99e0ebc3-09e8-47e6-be67-5743ca5e6d15" />

Alternatively, goals can be sent programmatically via the command line using ROS 2 action interface.

Below is an example command for setting a goal for tb1:

```
$ ros2 action send_goal /tb1/navigate_to_pose nav2_msgs/action/NavigateToPose \
'{
  pose: {
    header: {
      frame_id: "map"
    },
    pose: {
      position: {
        x: 2.0,
        y: 0.0,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
      }
    }
  }
}'
```

Replace /tb1 with the appropriate robot namespace (e.g., /tb2, /tb3, etc.) and modify the x, y, and orientation fields to specify the desired goal pose.

## RQT Usage
rqt is a versatile tool for inspecting various ROS 2 data. In a multi-robot setup where each robot operates within its own namespace, specific configurations are required for correct usage.
Ensure that the /tf and /tf_static topics are mapped using absolute names (prefixed with /). Additionally, launch rqt within the desired robot's namespace to correctly visualize and interact with its respective topics.

```
rqt --ros-args -r __ns:=/tb1 -r /tf:=tf -r /tf_static:=tf_static
```

<img width="1630" height="1572" alt="image" src="https://github.com/user-attachments/assets/a8f4221b-705e-4b52-ab04-03916a60de08" />


## 🐳 Running via Dockers
A Dockerfile is provided to simplify the setup and execution of multi-robot simulation using ROS 2 Jazzy. This enables running the project even on systems other than Ubuntu 24.04 (e.g. Ubuntu 22.04) without requiring ROS installation on the host. The Docker image includes all necessary dependencies preconfigured.

### 🛠️ Build Docker image

Clone the repository and build the Docker image:

```
$ git clone  https://github.com/arshadlab/tb3_multi_robot.git -b jazzy
$ cd docker
$ docker build -t tb3_multi_robot:jazzy .
```

### 🚀 Launch robots in Gazebo

Run the container and launch the Gazebo simulation:

```
$ docker run -it --rm  --name tb3sim   --env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --volume="/dev/dri:/dev/dri"   tb3_multi_robot:jazzy ros2 launch tb3_multi_robot tb3_world.launch.py
```

Ensure the command `xhost +local:docker` is executed on the host system to permit GUI display access for Docker containers.

### Launch Nav2 nodes in already running container

After the robots are active, open a new terminal and run:

```
$ docker exec -it tb3sim bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch tb3_multi_robot tb3_nav2.launch.py"
```

This launches the Nav2 stack inside the already running container.

## Robot Configuration

The placement and activation of individual robots are defined in the config/robots.yaml file. Each robot is assigned a unique name and initial position. Set the enabled flag to true to include a robot in the simulation or false to exclude it.

Below is an example configuration used in the Nav2 simulation:

```
 robots:
  - name: tb1
    x_pose: "-1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb2
    x_pose: "-1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
  - name: tb3
    x_pose: "1.5"
    y_pose: "-0.5"
    z_pose: 0.01
    enabled: true
  - name: tb4
    x_pose: "1.5"
    y_pose: "0.5"
    z_pose: 0.01
    enabled: false
```
💡 Robots can be enabled or disabled by updating the enabled field, and their starting poses can be configured by modifying the position values accordingly.

## Improving performance

Simulating multiple robots (especially 4 or more) can be demanding on system resources. Below are some suggestions to help optimize performance:

### 1. Limit Robot Count
Running fewer robots significantly reduces CPU and memory load. Consider limiting the simulation to 2 robots if performance is a concern.

### 2. Lower Simulation Update Rates
Reduce the update frequency in the Gazebo .world file to ease the physics computation load. In worlds/tb3_world.world, modify the <physics> block as follows:

```
<physics type="ode">
      <real_time_update_rate>100.0</real_time_update_rate>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
```

🔧 Lower real_time_update_rate and higher max_step_size lead to fewer simulation steps per second, improving runtime performance.

### 3. Reduce Topic Frequency
Consider modifying the robot model or relevant plugins to reduce the frequency of published topics (e.g., /odom, /tf, /scan) if they are not critical at high rates.

## 📎 Note on Included Files

Some of configuration and model files (e.g., from turtlebot3 and nav2) have been directly copied into this repository. These were modified to better suit the multi-robot simulation and to ensure long-term consistency and reproducibility—even if the original upstream repositories evolve or change in the future. All original credit for these files remains with their respective authors and maintainers.
