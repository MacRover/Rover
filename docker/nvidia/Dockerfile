ARG CUDA=10.2
FROM nvidia/cudagl:${CUDA}-devel-ubuntu18.04

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8

WORKDIR /root
# Install extra components
RUN apt-get update \
    && apt-get install -y build-essential lsb-release git wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-pip python3-pip ros-melodic-desktop-full ros-melodic-ros-controllers
# Copy the files into docker
COPY . Rover/
# Add ros commands to path then make the project and update path
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc \
    && . /opt/ros/melodic/setup.sh && cd /root/Rover/ROS_WS && catkin_make \
    && echo "source /root/Rover/ROS_WS/devel/setup.bash" >> /root/.bashrc \
    # Initialize rosdep
    && rosdep init \
    && rosdep update
# Launch (interactive must be specified to call ~/.bashrc)
CMD [ "/bin/bash", "-ic", "roscore" ]