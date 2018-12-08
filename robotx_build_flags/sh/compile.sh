#!/bin/bash
. /opt/ros/kinetic/setup.bash
mkdir -p $HOME/catkin_ws/src

cd $HOME/catkin_ws/src
catkin_init_workspace
git clone https://github.com/OUXT-Polaris/robotx_core.git
cd robotx_core
git checkout develop

cd $HOME/catkin_ws
wstool init src
wstool merge -t src src/robotx_core/dependencies.rosinstall
wstool up -t src
rosdep install -i -r -y --from-paths src --rosdistro kinetic

catkin_make

cd $HOME
tar zcvf catkin_ws.tar.gz catkin_ws
