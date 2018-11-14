cmake_minimum_required(VERSION 2.8.3)
project(robotx_pcl_tools)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  rviz
  robotx_msgs
  tf
  pcl_conversions
  pcl_msgs
  pcl_ros
)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)


add_definitions(-DQT_NO_KEYWORDS)

set(pcl_plane_removal_src src/pcl_plane_removal.cpp)
add_executable(pcl_plane_removal_node src/pcl_plane_removal_node.cpp ${pcl_plane_removal_src})
target_link_libraries(pcl_plane_removal_node ${catkin_LIBRARIES})

#############
## Install ##
#############
# CPP plugin
install(TARGETS pcl_plane_removal_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# Include files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# All files
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

foreach(dir config icons/classes launch resource srv)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
