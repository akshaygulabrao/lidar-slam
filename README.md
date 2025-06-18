# Point-LIO Performane on the Newer College Datasets
The repository aims to reproduce [LIDAR SLAM algorithms](https://arxiv.org/pdf/2311.00276) with the [Point-LIO](https://drive.google.com/file/d/1I8fByqJ-yE4lvYqeCvvjkzrPWRSjVYpg/view?usp=sharing) Algorithm. The code is provided so all that needs to be done is recording the output on the Newer College Dataset. I recommend using Foxglove Studio and Docker on macOS to run the experiments. See [Installing ROS1 on macOS with Docker](https://foxglove.dev/blog/installing-ros1-on-macos-with-docker) for assistance.

## Reproducing Point-LIO on collection 1
The python script **run_all** runs point-LIO on all instances of Newer College Dataset - Collection 1. It runs each bag inside a docker container and uses the library [evo_traj] to compare it against the provided ground-truth data. 


## In Progress

nc-quad-easy, 0.068729
nc-quad-medium, 0.057268
nc-quad-hard, 0.057049
stairs, 120.619195
