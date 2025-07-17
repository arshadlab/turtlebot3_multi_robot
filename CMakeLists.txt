cmake_minimum_required(VERSION 3.5)
project(tb3_multi_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(multi_robot_scripts)

install(
  PROGRAMS
  multi_robot_scripts/turtlebot3_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

