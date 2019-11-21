cmake_minimum_required(VERSION 2.8.3)
project(welder_node_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
