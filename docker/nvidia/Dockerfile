ARG CUDA=10.2
FROM nvidia/cudagl:${CUDA}-devel-ubuntu18.04

# ARGs are lost after each "FROM" statement
ARG CUDA=10.2 
ARG COMPUTE_CAPABILITY=5.2
ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8

WORKDIR /root
# Install ROS
RUN apt-get update \
    && apt-get install -y build-essential lsb-release git wget \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
    && apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool python-pip python3-pip ros-melodic-desktop-full 
# Install other dependencies
RUN apt-get update \
    && apt-get install -y libcudnn8=8.0.3.33-1+cuda${CUDA}
# Copy in cudnn.deb
# This (cuDNN Developer Library for Ubuntu18.04) must be downloaded from https://developer.nvidia.com/cudnn-download-survey
# Rename to "cudnn.deb" and place in the docker folder of this repo
COPY docker/cudnn.deb cudnn.deb
# Install cuDNN
RUN dpkg -i cudnn.deb
# Build and install OpenCV from source
RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir build && \
    cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_PYTHON_EXAMPLES=OFF -D INSTALL_C_EXAMPLES=OFF -D OPENCV_ENABLE_NONFREE=ON -D WITH_CUDA=ON -D WITH_CUDNN=ON -D OPENCV_DNN_CUDA=ON -D ENABLE_FAST_MATH=1 -D CUDA_FAST_MATH=1 -D CUDA_ARCH_BIN=${COMPUTE_CAPABILITY} -D WITH_CUBLAS=1 -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules -D BUILD_EXAMPLES=OFF -D OPENCV_GENERATE_PKGCONFIG=ON .. && \
    make -j4 && \
    make install
# Download darknet and weights
# Upon startup modify Makefile and build darknet
RUN git clone https://github.com/AlexeyAB/darknet.git && \
    cd darknet/cfg && \
    wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
# Install extra ROS components
RUN apt-get update \
    && apt-get install -y \
        ros-melodic-joy \
        ros-melodic-ros-controllers \
        ros-melodic-rtabmap-ros
# Copy files into docker
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