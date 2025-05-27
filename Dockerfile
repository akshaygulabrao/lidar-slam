FROM ros:noetic-ros-core
RUN apt-get update && apt-get install -y \
    ros-noetic-foxglove-bridge
RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-conversions
RUN apt-get update && apt-get install -y \
    libeigen3-dev
RUN apt-get update && apt-get install -y \
    git
RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-ros
RUN apt-get update && apt-get install -y \
    ros-noetic-eigen-conversions
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev
COPY ./catkin_ws /catkin_ws
WORKDIR /catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash; catkin_make"
RUN apt-get update && apt-get install -y \
    vim