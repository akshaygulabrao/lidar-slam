FROM ros:noetic-ros-core
RUN apt-get update && apt-get install -y \
    ros-noetic-foxglove-bridge \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    libeigen3-dev \
    libgoogle-glog-dev \
    git \
    vim && \
    rm -rf /var/lib/apt/lists/*

