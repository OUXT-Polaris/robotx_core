cmake_minimum_required(VERSION 2.8.3)
project(robotx_build_flags)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CFG_EXTRAS toolchain.cmake)