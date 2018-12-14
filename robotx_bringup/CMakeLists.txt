cmake_minimum_required(VERSION 2.8.3)
project(robotx_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  robotx_decision_maker
  robotx_driver
  robotx_msgs
  robotx_navigation
  robotx_recognition
  robotx_twist_controller
  wamv_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotx_bringup
#  CATKIN_DEPENDS robotx_decision_maker robotx_driver robotx_msgs robotx_navigation robotx_recognition robotx_twist_controller wamv_description
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)