FROM multiarch/ubuntu-core:arm64-xenial

RUN apt-get update && apt-get install -y curl build-essential

RUN curl -o pylon_5.1.0.12682-deb0_arm64.deb https://www.baslerweb.com/fp-1535524598/media/downloads/software/pylon_software/pylon_5.1.0.12682-deb0_arm64.deb \
  && dpkg --force-all -i pylon_5.1.0.12682-deb0_arm64.deb\
  && rm pylon_5.1.0.12682-deb0_arm64.deb

RUN curl https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb -o cuda_arm64.deb \
   && dpkg -i cuda_arm64.deb \
   && rm cuda_arm64.deb \
   && cd /var/cuda-repo-9-0-local/ \
   && dpkg --force-all -i *.deb

RUN curl -o cudnn.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/libcudnn7_7.0.5.15-1+cuda9.0_arm64.deb \
  && dpkg -i cudnn.deb \
  && rm cudnn.deb

RUN curl -o cudnn-dev.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/libcudnn7-dev_7.0.5.15-1+cuda9.0_arm64.deb \
  && dpkg -i cudnn-dev.deb \
  && rm cudnn-dev.deb

RUN curl -o nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.2.1/m8u2ki/JetPackL4T_321_b23/nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && dpkg -i nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && rm nv-tensorrt-repo-ubuntu1604-ga-cuda9.0-trt3.0.4-20180208_1-1_arm64.deb \
 && cd /var/nv-tensorrt-repo-ga-cuda9.0-trt3.0.4-20180208/ \
 && dpkg --force-all -i *.deb

RUN apt-get update && apt-get -f install -y
RUN apt-get install -y lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install -y ros-kinetic-ros-base git
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN apt-get install -y python-rosinstall
RUN rosdep init && rosdep update
RUN mkdir -p /home/catkin_ws/src
WORKDIR /home/catkin_ws/src
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && catkin_init_workspace"]
WORKDIR /home/catkin_ws/
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && catkin_make"]
WORKDIR /home/catkin_ws/src
RUN git clone https://github.com/OUXT-Polaris/robotx_core.git
WORKDIR /home/catkin_ws/src/robotx_core
RUN git checkout develop
WORKDIR /home/catkin_ws
RUN wstool init src
RUN wstool merge -t src src/robotx_core/dependencies.rosinstall
RUN wstool up -t src
RUN rosdep install -i -r -y --from-paths src --rosdistro kinetic
RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && catkin_make"]
