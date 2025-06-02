# Point-LIO Performane on the Newer College Datasets
The repository aims to reproduce [LIDAR SLAM algorithms](https://arxiv.org/pdf/2311.00276). Algorithm performance will be measured by RMS Trajectory Error and RMS Heading Error.

Currently, the first algorithm being tested is [Point LIO](https://github.com/hku-mars/Point-LIO).The algorithm outputs the localization to the /tf topic.

I recommend using Foxglove Studio and Docker on macOS to run the experiments. See [Installing ROS1 on macOS with Docker](https://foxglove.dev/blog/installing-ros1-on-macos-with-docker) for assistance.

## Reproducing Point-LIO
See the dockerfile for the required packages needed to to build [Point-LIO](https://github.com/hku-mars/Point-LIO). Currently, there are 3 terminals that are required to run the script. 

In terminal 1, start **roscore**
```bash
docker run --rm --name roscore --hostname roscore --network rosnet rosdemo roscore 
```

In terminal 2, start recording the output of the SLAM script. Starting after the SLAM algorithm is launched will cause dropped frames resulting in reduced accuracy.
```bash

```

This sets up the docker containers necessary, forwards the output to foxglove studio, and records the path in /bags/recorded_data.bag.


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

