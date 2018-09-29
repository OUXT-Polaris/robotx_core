cmake_minimum_required(VERSION 2.8.3)
project(robotx_bringup)
find_package(catkin REQUIRED)

catkin_package()

# Install
set(LAUNCH_DIRS launch launch/sensor launch/driver)
set(CONFIG_DIRS config/velodyne)

foreach(dir ${CONFIG_DIRS} ${LAUNCH_DIRS} )
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/sensor)
endif()
