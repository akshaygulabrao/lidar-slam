# Point-LIO Performane on the Newer College Datasets
The repository aims to reproduce [LIDAR SLAM algorithms](https://arxiv.org/pdf/2311.00276) with the [Point-LIO](https://drive.google.com/file/d/1I8fByqJ-yE4lvYqeCvvjkzrPWRSjVYpg/view?usp=sharing) Algorithm. The code is provided so all that needs to be done is recording the output on the Newer College Dataset. I recommend using Foxglove Studio and Docker on macOS to run the experiments. See [Installing ROS1 on macOS with Docker](https://foxglove.dev/blog/installing-ros1-on-macos-with-docker) for assistance.

## Reproducing Point-LIO
The script can be run on any arbitrary bag file using `launch_rosScript.sh`. The script takes in 3 arguments: 
1. run name
2. bag name
3. tum file name - used to measure gt

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

## Getting Started

To benchmark and visualize the data, I use the [evo](https://github.com/MichaelGrupp/evo) library. I ran into installation issues with tkinter, so I had install PyQt5 as the repo instructions suggest. The commands below can be used to visualize the ground truth data.

```bash
uv run evo_config set plot_backend Qt5Agg
uv run evo_traj bag bags/recorded_data.bag /aft_mapped_to_init --save_as_tum
uv run evo_traj tum aft_mapped_to_init.tum --ref ../collection1-newercollege/ground_truth/tum_format/gt-nc-quad-easy.csv -p --plot_mode=xy --align
uv run evo_rpe tum ../collection1-newercollege/ground_truth/tum_format/gt-nc-quad-easy.csv aft_mapped_to_init.tum
```

## In Progress
Finish benchmarking all Newer-College rosbags with LIDAR.