cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)
catkin_package(CATKIN_DEPENDS xacro)

# Xacro build & install
xacro_add_files(
  urdf/wamv_basic.xacro
    INORDER INSTALL DESTINATION urdf
)

#############
## Install ##
#############
set(URDF_DIRS urdf urdf/base urdf/engine urdf/ball_launcher urdf/sensors urdf/sensors/camera urdf/sensors/gps urdf/sensors/imu urdf/sensors/twist urdf/sensors/velodyne urdf/sensors/stereo_camera)
foreach(dir launch mesh ${URDF_DIRS})
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
