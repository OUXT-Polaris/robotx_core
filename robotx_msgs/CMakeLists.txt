cmake_minimum_required(VERSION 2.8.3)
project(robotx_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  jsk_recognition_msgs
  #geographic_msgs
  robotx_build_flags
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BallLauncherStatus.msg
  Heartbeat.msg
  TechnicalDirectorNetworkStatus.msg
  UsvDrive.msg
  RegionOfInterest2D.msg
  RegionOfInterest2DArray.msg
  ObjectRegionOfInterestArray.msg
  ObjectRegionOfInterest.msg
  ObjectType.msg
  #GeographicPoint.msg
  #GeographicLine.msg
  #GeographicMap.msg
  #WayPointArray.msg
  #WayPoint.msg
  #NavigationStatus.msg
  Event.msg
  State.msg
  StateChanged.msg
  CoastLine.msg
  CoastLineArray.msg
  ObstacleMap.msg
  FieldMap.msg
  EntranceAndExitGatesReport.msg
  IdentifySymbolsAndDockReport.msg
  DetectAndDeliverReport.msg
  ScanTheCodeReport.msg
  CarrotPlannerConfigure.msg
  ObstacleAvoidConfigure.msg
  SignalLamp.msg
  PowerStatus.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#
  sensor_msgs#
  std_msgs
  jsk_recognition_msgs
  #geographic_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
  jsk_recognition_msgs
  #geographic_msgs
)

###########
## Build ##
###########

# Nothing to build

#############
## Install ##
#############
foreach(dir msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

# Nothing to test
