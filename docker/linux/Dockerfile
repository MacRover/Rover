#OPTIONAL -- This file helps members get started quickly with the rover ROS stack
FROM osrf/ros:melodic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8

WORKDIR /root
# Install extra components
RUN apt-get update \
    && apt-get install -y \
        ros-melodic-joy \
        ros-melodic-ros-controllers \
        ros-melodic-rtabmap-ros
# Copy the files into docker
COPY . Rover/
# Add ros commands to path then make the project and update path
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc \
    && . /opt/ros/melodic/setup.sh \
    # Initialize rosdep
    # && rosdep init \
    # && rosdep update \
    && cd /root/Rover/ROS_WS && catkin_make \
    && echo "source /root/Rover/ROS_WS/devel/setup.bash" >> /root/.bashrc
# Launch (interactive must be specified to call ~/.bashrc)
CMD [ "/bin/bash", "-ic", "roscore" ]