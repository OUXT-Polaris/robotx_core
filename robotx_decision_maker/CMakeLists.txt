cmake_minimum_required(VERSION 2.8.3)
project(robotx_decision_maker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  robotx_msgs
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robotx_decision_maker
  CATKIN_DEPENDS robotx_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
