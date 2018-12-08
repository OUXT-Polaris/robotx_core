#!/bin/bash
# ros setup
. /opt/ros/kinetic/setup.bash

# refresh
rm -rf $HOME/catkin_ws
rm -rf $HOME/catkin_ws.tar.gz

# create catkin workspace
mkdir -p $HOME/catkin_ws/src

cd $HOME/catkin_ws/src
catkin_init_workspace
git clone https://github.com/OUXT-Polaris/robotx_core.git
cd robotx_core
git checkout develop

# rosdep
cd $HOME/catkin_ws
wstool init src
wstool merge -t src src/robotx_core/dependencies.rosinstall
wstool up -t src
rosdep install -i -r -y --from-paths src --rosdistro kinetic

# compile
catkin_make -j16

# package built files as tar
cd $HOME
tar zcvf catkin_ws.tar.gz catkin_ws
