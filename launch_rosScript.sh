export WORKDIR="/Users/trading/workspace/lidar-slam"

if [ $# -eq 0 ]; then
    echo "Usage: $0 <ros script>"
    exit 1
fi

docker build . -t rosdemo && \
docker run --rm -it\
  --network rosnet \
  -v /Users/trading/workspace/collection1-newercollege/2021-07-01-10-37-38-quad-easy.bag:/test.bag \
  -v $WORKDIR/catkin_ws/:/catkin_ws/ \
  -v $WORKDIR/pointLIO_foxglove.launch:/pointLIO_foxglove.launch \
  -v $WORKDIR/bags:/bags \
  -v $WORKDIR/$1:/$1 \
  -p 8765:8765 \
  --name pointLIO \
  rosdemo \
  /bin/bash $1

uv run evo_traj bag bags/recorded_data.bag /aft_mapped_to_init --save_as_tum
uv run evo_traj tum aft_mapped_to_init.tum --ref ../collection1-newercollege/ground_truth/tum_format/gt-nc-quad-easy.csv -p --plot_mode=xy --align
uv run evo_rpe tum ../collection1-newercollege/ground_truth/tum_format/gt-nc-quad-easy.csv aft_mapped_to_init.tum -a