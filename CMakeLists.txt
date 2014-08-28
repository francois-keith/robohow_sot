cmake_minimum_required(VERSION 2.4.6)
project(robohow_sot)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  robohow_common_msgs
  dynamic_graph_bridge_msgs
 )

catkin_python_setup()

catkin_package()

