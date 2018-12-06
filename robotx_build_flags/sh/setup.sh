#!/bin/bash
sudo apt-get update && sudo apt-get install -y curl build-essential

cd

curl -o pylon_5.1.0.12682-deb0_arm64.deb https://www.baslerweb.com/fp-1535524598/media/downloads/software/pylon_software/pylon_5.1.0.12682-deb0_arm64.deb \
  && sudo dpkg --force-all -i pylon_5.1.0.12682-deb0_arm64.deb \
  && rm pylon_5.1.0.12682-deb0_arm64.deb

curl https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb -o cuda_arm64.deb \
  && sudo dpkg -i cuda_arm64.deb \
  && rm cuda_arm64.deb \
  && cd /var/cuda-repo-9-0-local/ \
  && sudo dpkg --force-all -i *.deb

curl -o cudnn.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/libcudnn7_7.0.5.15-1+cuda9.0_arm64.deb \
  && sudo dpkg -i cudnn.deb \
  && rm cudnn.deb

curl -o cudnn-dev.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/libcudnn7-dev_7.0.5.15-1+cuda9.0_arm64.deb \
  && sudo dpkg -i cudnn-dev.deb \
  && rm cudnn-dev.deb

curl -o nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && sudo dpkg -i nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && rm nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && cd /var/nv-tensorrt-repo-ga-cuda9.0-trt3.0.4-20180208/ \
 && sudo dpkg --force-all -i *.deb

sudo apt-get update && sudo apt-get -f install -y
sudo apt-get install -y lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base git python-rosinstall
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
sudo rosdep init && rosdep update
