The ROS2 project  scalable solution for launching multiple TurtleBot3 robots with navigation capabilities using the Navigation2 (Nav2) stack. By leveraging namespaces in ROS2, this project enables the seamless deployment of multiple TurtleBot3 robots in a simple and organized manner. Each robot instance can be differentiated by its unique namespace, ensuring independence and preventing naming conflicts.

## Run without nav2 stack

ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py enable_drive:=True

# turtlebot3_multi_robot
![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/fc958709-018d-48d2-b5b6-6674b53913c8)

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/c955b964-27fe-46d4-8696-d3c0d106dbe0)

## Run with nav2 stack

ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py enable_drive:=True  use_sim_time:=True

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/621f8884-1cd4-4eab-8ab4-50c1fd42d13b)


Rviz2 output for first robot
![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/0c3eaae5-74f0-40e8-be80-91bcf2266a4a)

Rviz2 output for all 4 robots
![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/e3ae59a2-ddae-4c80-8232-2d06d053b3e8)
