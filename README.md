# LIDAR SLAM
The repository aims to reproduce [LIDAR SLAM algorithms](https://arxiv.org/pdf/2311.00276). Algorithm performance will be measured by RMS Trajectory Error and RMS Heading Error.

Currently, the first algorithm being tested is [Point LIO](https://github.com/hku-mars/Point-LIO). The issue I'm currently facing is that the algorithm is implemented as a ROS-Noetic package which depends on Ubuntu 20.04. Creating a Dockerfile enhances the reproducibility of this algorithm.

Foxglove Studio published a useful article on this issue [Installing ROS1 on macOS with Docker](https://foxglove.dev/blog/installing-ros1-on-macos-with-docker). DO NOT try to wrap this into a docker-compose. I have wasted ~30 hours on this and ran into issues with keeping the containers running. You can look into the commit history to see my last attempts.

## Setup
The following setup is only tested on macOS.

Remember to set display frame to os_sensor.



``` bash
docker network create rosnet
docker run --rm --name roscore --hostname roscore --network rosnet -it rosdemo roscore
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' rosdemo rostopic list
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' rosdemo rostopic pub /chatter std_msgs/String 'data: hello' -r 1
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' rosdemo rostopic echo /chatter
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' -p 8765:8765 rosdemo roslaunch foxglove_bridge foxglove_bridge.launch
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' -v /Users/ox/workspace/newer-college-1/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag rosdemo rosbag play 2021-07-01-10-37-38-quad-easy.bag
```

```bash
docker run --rm -it --network rosnet --env 'ROS_MASTER_URI=http://roscore:11311/' -v /Users/ox/workspace/newer-college-1/2021-07-01-10-37-38-quad-easy.bag:/2021-07-01-10-37-38-quad-easy.bag rosdemo bash
```


## Visualizing Ground Truth Data
A LIDAR streams (x,y,z,r) egocentric measurements. An IMU streams acceleration in (x,y,z) egocentric. By pairing both together, it is possible to generate accurate voxels of the surrounding area.

This repository parses the ground truth data of the [Newer College Dataset Multicam](https://ori-drs.github.io/newer-college-dataset/multi-cam/).

It assumes that you have a directory like so:

```
collection_1/ground_truth/
├── gt-nc-quad-easy.csv
├── gt-nc-quad-hard.csv
├── gt-nc-quad-medium.csv
└── gt-nc-stairs.csv
```

Replace the filename on line 6 in main.py to the correct path of the csv file. I recommend using [ROS 2 Docker](https://foxglove.dev/blog/installing-ros2-on-macos-with-docker)
along with [Foxglove Studio](https://app.foxglove.dev/).

The dataset proposes the following research topics:
1. LIDAR SLAM
2. Visual Appearance-based Loop Closure
3. 3D Lidar Reconstruction
4. Visual Odometry

## Getting Started

To benchmark and visualize the data, I use the [evo](https://github.com/MichaelGrupp/evo) library. I ran into installation issues with tkinter, so I had install PyQt5 as the repo instructions suggest. The commands below can be used to visualize the ground truth data.

```bash
uv run evo_config set plot_backend Qt5Agg
uv run parse_tum.py
uv run evo_traj tum gt-nc-quad-easy.tum -p --plot_mode=xy
```
```bash
docker compose build && docker compose run --rm --remove-orphans  cartographer-ros
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```

## Reproducing Point-LIO
See the dockerfile for the required packages needed to to build [Point-LIO](https://github.com/hku-mars/Point-LIO). After installing the dockerfile, source the launch files with 
```bash
source devel/setup.bash
```
Then follow 5.3 to configure for the Ouster LIDAR (in progress). 
