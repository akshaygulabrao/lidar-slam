export WORKDIR="/Users/trading/workspace/lidar-slam"

docker build . -t rosdemo && \
docker run --rm -it \
  --network rosnet \
  -v /Users/trading/workspace/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/pointLIO_foxglove.launch:/pointLIO_foxglove.launch \
  -v $WORKDIR/a.csv:/a.csv \
  -v $WORKDIR/bags:/bags \
  --env ROS_MASTER_URI=http://roscore:11311 \
  rosdemo \
  bash