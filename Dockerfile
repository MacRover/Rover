#OPTIONAL -- This file helps members get started quickly with the rover ROS stack
FROM osrf/ros:melodic-desktop-full
WORKDIR /root
# Copy the files into docker
COPY . rover/
# Add ros commands to path
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
# Make the project and update path
RUN . /opt/ros/melodic/setup.sh && cd /root/rover/ROS_WS && catkin_make && echo "source /root/rover/ROS_WS/devel/setup.bash" >> /root/.bashrc
# Launch 
CMD [ "roscore" ]