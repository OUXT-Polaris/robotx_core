cmake_minimum_required(VERSION 2.8.3)
project(wamv_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)
catkin_package(CATKIN_DEPENDS xacro)

# Xacro build & install
xacro_add_files(
  urdf/wamv_basic.urdf.xacro
  urdf/wamv_robotx.urdf.xacro
    INORDER INSTALL DESTINATION urdf
)

#############
## Install ##
#############
foreach(dir launch mesh urdf)
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
