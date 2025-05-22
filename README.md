# LIDAR SLAM

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
roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765
```
## Reproducing Point-LIO

I'm 

## Troubleshooting

 ```bash
Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: runc create failed: unable to start container process: exec: "/ros_entrypoint.sh": permission denied: unknown
```
