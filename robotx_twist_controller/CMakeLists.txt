cmake_minimum_required(VERSION 2.8.3)
project(robotx_twist_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  robotx_build_flags
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robotx_twist_controller src/robotx_twist_controller.cpp)
target_link_libraries(robotx_twist_controller ${catkin_LIBRARIES})

foreach(programs robotx_twist_controller)
  install(TARGETS ${programs}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endforeach(nodes)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
