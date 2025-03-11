## Multiple Turtlebot3 robot support in Gazebo
The ROS2 project  scalable solution for launching multiple TurtleBot3 robots with navigation capabilities using the Navigation2 (Nav2) stack. By leveraging namespaces in ROS2, this project enables the seamless deployment of multiple TurtleBot3 robots in a simple and organized manner. Each robot instance can be differentiated by its unique namespace, ensuring independence and preventing naming conflicts.


The 'master' branch includes an implementation that functions with the humble framework, while the 'foxy' branch provides support specifically for ROS2 Foxy.

'master' -> ROS2 Humble

'foxy' -> ROS2 Foxy

The code in the "foxy" branch is compatible with ROS2 humble. In the master branch, there is an updated launch file for bringing up nav2 with composite nodes. However, the creation of composite nodes is currently disabled due to an issue in the ROS2 humble implementation. This issue pertains to the propagation of namespace mapping to nodes (in composite container) with sub-namespaces, such as "/global_costmap/global_costmap".

## Run without nav2 stack
**Guide**: https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620
```
ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py enable_drive:=True
```
# turtlebot3_multi_robot

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/fc958709-018d-48d2-b5b6-6674b53913c8)

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/c955b964-27fe-46d4-8696-d3c0d106dbe0)

## Run with nav2 stack
**Guide**: https://medium.com/@arshad.mehmood/a-guide-to-multi-robot-navigation-utilizing-turtlebot3-and-nav2-cd24f96d19c6

#### Robot Configuration

The arrangement of robots is configured in gazebo_multi_nav2_world.launch.py launch file. A potential future enhancement could involve retrieving the configurations from a file, such as json.

Names and poses for the robots in nav2 example
```
 robots = [
 {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 # …
 # …
 ]
```
```
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=True  use_sim_time:=True
```
![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/621f8884-1cd4-4eab-8ab4-50c1fd42d13b)


Rviz2 output for first robot

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/0c3eaae5-74f0-40e8-be80-91bcf2266a4a)

Rviz2 output for all 4 robots

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/e3ae59a2-ddae-4c80-8232-2d06d053b3e8)
