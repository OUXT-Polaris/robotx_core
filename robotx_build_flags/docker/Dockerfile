FROM multiarch/ubuntu-core:arm64-xenial

RUN apt-get update && apt-get install -y curl

RUN curl -o pylon_5.1.0.12682-deb0_arm64.deb https://www.baslerweb.com/fp-1535524598/media/downloads/software/pylon_software/pylon_5.1.0.12682-deb0_arm64.deb \
  && dpkg --force-all -i pylon_5.1.0.12682-deb0_arm64.deb\
  && rm pylon_5.1.0.12682-deb0_arm64.deb

RUN curl https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb -o cuda_arm64.deb \
   && dpkg -i cuda_arm64.deb \
   && rm cuda_arm64.deb \
   && cd /var/cuda-repo-9-0-local/ \
   && dpkg --force-all -i *.deb

RUN curl -o cudnn.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39//libcudnn7_7.1.5.14-1+cuda9.0_arm64.deb \
  && dpkg -i cudnn.deb \
  && rm cudnn.deb

RUN curl -o cudnn-dev.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39//libcudnn7-dev_7.1.5.14-1+cuda9.0_arm64.deb \
  && dpkg -i cudnn-dev.deb \
  && rm cudnn-dev.deb 

RUN curl -o libnvinfer4_4.1.3-1+cuda9.0_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/libnvinfer4_4.1.3-1+cuda9.0_arm64.deb \
  && dpkg -i libnvinfer4_4.1.3-1+cuda9.0_arm64.deb \
  && rm libnvinfer4_4.1.3-1+cuda9.0_arm64.deb

RUN curl -o libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb \
  && dpkg -i libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb \
  && rm libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb


RUN curl -o libnvinfer-samples_4.1.3-1+cuda9.0_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/libnvinfer-samples_4.1.3-1+cuda9.0_arm64.deb \
  && dpkg -i libnvinfer-samples_4.1.3-1+cuda9.0_arm64.deb \
  && rm libnvinfer-samples_4.1.3-1+cuda9.0_arm64.deb

RUN curl -o tensorrt_4.0.2.0-1+cuda9.0_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/tensorrt_4.0.2.0-1+cuda9.0_arm64.deb \
  && dpkg -i tensorrt_4.0.2.0-1+cuda9.0_arm64.deb \
  && rm tensorrt_4.0.2.0-1+cuda9.0_arm64.deb

RUN curl -o libgie-dev_4.1.3-1+cuda9.0_arm64.deb https://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/3.3/lw.xd42/JetPackL4T_33_b39/libgie-dev_4.1.3-1+cuda9.0_arm64.deb \
  && dpkg -i libgie-dev_4.1.3-1+cuda9.0_arm64.deb \
  && rm libgie-dev_4.1.3-1+cuda9.0_arm64.deb

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