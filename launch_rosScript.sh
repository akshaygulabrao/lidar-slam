export WORKDIR="/Users/trading/workspace/lidar-slam"

if [ $# -eq 0 ]; then
    echo "Usage: $0 <ros script>"
    exit 1
fi

docker build . -t rosdemo && \
docker run --rm -it\
  --network rosnet \
  -v /Users/trading/workspace/collection1-newercollege/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/pointLIO_foxglove.launch:/pointLIO_foxglove.launch \
  -v $WORKDIR/bags:/bags \
  -v $WORKDIR/$1:/$1 \
  -p 8765:8765 \
  --env ROS_MASTER_URI=http://roscore:11311 \
  --name pointLIO \
  rosdemo \
  /bin/bash $1