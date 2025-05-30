export WORKDIR="/Users/trading/workspace/lidar-slam"

docker build . -t rosdemo && \
docker run --rm \
  --network rosnet \
  -v /Users/trading/workspace/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/pointLIO_foxglove.launch:/pointLIO_foxglove.launch \
  -p 8765:8765 \
  --env ROS_MASTER_URI=http://roscore:11311 \
  rosdemo \
  /bin/bash -c "source /setup_ros.sh && roslaunch pointLIO_foxglove.launch"