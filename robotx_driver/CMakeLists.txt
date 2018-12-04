cmake_minimum_required(VERSION 2.8.3)
project(robotx_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  robotx_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
  diagnostic_updater
  nmea_msgs
  robotx_build_flags
)

find_package(Boost REQUIRED COMPONENTS thread system chrono)

find_package(CUDA)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotx_driver
  CATKIN_DEPENDS joy robotx_msgs roscpp rospy sensor_msgs std_msgs diagnostic_msgs diagnostic_updater
#  DEPENDS system_lib
)

if(CUDA_FOUND)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
  )
else()
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
endif()

set(heartbeat_publisher_node_src src/heartbeat_publisher_node.cpp src/heartbeat_publisher.cpp src/tcp_client.cpp)
add_executable(heartbeat_publisher_node ${heartbeat_publisher_node_src})
add_dependencies(heartbeat_publisher_node robotx_msgs_generate_messages_cpp)
target_link_libraries(heartbeat_publisher_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_document(heartbeat_publisher_node)
#clang_format(heartbeat_publisher_node)

set(robotx_hardware_interface_src 
  src/robotx_hardware_interface_node.cpp src/robotx_hardware_interface.cpp src/tcp_client.cpp src/remote_operated_interface.cpp
  src/joy_analyzer.cpp)
add_executable(robotx_hardware_interface_node ${robotx_hardware_interface_src} src/robotx_hardware_interface_node.cpp)
add_dependencies(robotx_hardware_interface_node robotx_msgs_generate_messages_cpp)
target_link_libraries(robotx_hardware_interface_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_document(robotx_hardware_interface_node)
#clang_format(robotx_hardware_interface_node)

add_executable(dynamixel_driver_node src/dynamixel_driver_node.cpp src/dynamixel_driver.cpp)
add_dependencies(dynamixel_driver_node robotx_msgs_generate_messages_cpp)
target_link_libraries(dynamixel_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(sc30_driver_node src/sc30_driver_node.cpp src/sc30_driver.cpp)
add_dependencies(sc30_driver_node robotx_msgs_generate_messages_cpp)
target_link_libraries(sc30_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(signal_lamp_driver_node src/signal_lamp_driver_node.cpp src/signal_lamp_driver.cpp src/tcp_client.cpp)
add_dependencies(signal_lamp_driver_node robotx_msgs_generate_messages_cpp)
target_link_libraries(signal_lamp_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(battery_monitor_driver_node src/battery_monitor_driver_node.cpp src/battery_monitor_driver.cpp src/tcp_server.cpp)
#add_dependencies(signal_lamp_driver_node robotx_msgs_generate_messages_cpp)
target_link_libraries(battery_monitor_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

message(STATUS "")
message(STATUS "FoundCUDA              : ${CUDA_FOUND}")

if(CUDA_FOUND)
  #set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -std=c++11 -gencode arch=compute_30,code=sm_30)
  #set(CUDA_SEPARABLE_COMPILATION ON)
  #set(CUDA_PROPAGATE_HOST_FLAGS OFF)
  #set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS}; "-std=c++11 -O3")
  set(CUDA_HOST_COMPILER g++)

  #set(cuda_diagnostic_src src/cuda_diagnostic.cpp)
  #cuda_add_executable(cuda_diagnostic_node ${cuda_diagnostic_src} src/cuda_diagnostic_node.cpp)
  #add_dependencies(cuda_diagnostic_node robotx_msgs_generate_messages_cpp)
  #target_link_libraries(cuda_diagnostic_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${CUDA_LIBRARIES})
  #add_document(cuda_diagnostic_node)
  #clang_format(cuda_diagnostic_node)
endif()


#############
## Install ##
#############
if(CUDA_FOUND)
  install(TARGETS robotx_hardware_interface_node heartbeat_publisher_node dynamixel_driver_node 
    sc30_driver_node signal_lamp_driver_node battery_monitor_driver_node #cuda_diagnostic_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  install(TARGETS robotx_hardware_interface_node heartbeat_publisher_node dynamixel_driver_node 
    sc30_driver_node signal_lamp_driver_node battery_monitor_driver_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

foreach(dir config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if(CUDA_FOUND)
  foreach(dir launch launch_cuda)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach(dir)
else()
  foreach(dir launch)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endforeach(dir)
endif()

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()