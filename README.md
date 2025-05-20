# LIDAR SLAM

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

To benchmark and visualize the data, I use the [evo](https://github.com/MichaelGrupp/evo) library. I ran into installation issues with tkinter, so I had install PyQt5 as the repo instructions suggest.

```bash
uv run evo_config set plot_backend Qt5Agg
uv run python main.py
uv run evo_traj tum proper_tum_format.tum -p --plot_mode=xy
```