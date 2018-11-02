cmake_minimum_required(VERSION 2.8.3)
project(robotx_decision_maker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  robotx_msgs
  roscpp
  std_msgs
  roslib
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

add_executable(robotx_state_machine_node src/robotx_state_machine_node.cpp 
  src/state_machine.cpp src/control_state_machine.cpp src/mission_state_machine.cpp
  src/navigation_state_machine.cpp src/rostate_machine.cpp)
target_link_libraries(robotx_state_machine_node ${catkin_LIBRARIES})
add_dependencies(robotx_state_machine_node robotx_msgs_generate_messages_cpp)

set(NODE_FILES robotx_state_machine_node)
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir launch data)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()