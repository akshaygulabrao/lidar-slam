docker build . -t rosdemo && \
docker run --rm -it \
  --network rosnet \
  -v /Users/trading/workspace/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag \
  -v /Users/trading/workspace/lidar-slam/catkin_ws/:/catkin_ws/ \
  -v /Users/trading/workspace/lidar-slam/pointLIO_foxglove.launch:/pointLIO_foxglove.launch \
  -p 8765:8765 \
  --env ROS_MASTER_URI=http://roscore:11311 \
  rosdemo \
  roslaunch pointLIO_foxglove.launch