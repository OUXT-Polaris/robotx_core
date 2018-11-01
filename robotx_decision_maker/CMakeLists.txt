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

add_executable(robotx_decision_maker_node src/robotx_decision_maker_node.cpp src/control_state_machine.cpp src/state_machine.cpp)
target_link_libraries(robotx_decision_maker_node ${catkin_LIBRARIES})
add_dependencies(robotx_decision_maker_node robotx_msgs_generate_messages_cpp)

set(NODE_FILES robotx_decision_maker_node)
foreach(nodes ${NODE_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endforeach(nodes)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()