# Point-LIO Performane on the Newer College Datasets



The repository aims to reproduce [LIDAR SLAM algorithms](https://arxiv.org/pdf/2311.00276) with the [Point-LIO](https://drive.google.com/file/d/1I8fByqJ-yE4lvYqeCvvjkzrPWRSjVYpg/view?usp=sharing) Algorithm. The code is provided so all that needs to be done is recording the output on the Newer College Dataset. I recommend using Foxglove Studio and Docker on macOS to run the experiments. See [Installing ROS1 on macOS with Docker](https://foxglove.dev/blog/installing-ros1-on-macos-with-docker) for assistance.

An important part of robotics is odometry. Odometry tracks the change of position between sensor readings. **Li**ght **D**etection and **R**anging (**LIDAR**) is an extremely popular sensor due to the depth accuracy. Point-LIO is a modern odometry algorithm that uses an **E**xtended **K**alman **Filter** (EKF) to merge noisy **I**nertial **M**ass **U**nit measurements with LiDAR readings. It improves on previous SOTA with 2 things:
1. point-by-point LiDAR updates instead of cloud updates, removing a systemic source of error
2. stochastic-process updated kinematic model which improves performance under IMU saturation

## Evaluation  
The python script **run_all** runs point-LIO on all instances of Newer College Dataset - Collection 1. It runs each bag inside a docker container and uses the library [evo_traj] to compare it against the provided ground-truth data. The quality of the measurement is measured with the root mean square error (RMSE) of the absolute pose error (APE). 

### Ground Truth

The ground truth data in [generated in two steps](https://ori-drs.github.io/newer-college-dataset/ground-truth/) . First, extremely accurate point clouds are taken of the full scene and Iterative Closest Point (ICP) is used to find the match the most stable inliers between frames. Then during the walk, the Point Cloud is matched against the accurate point cloud and an accurate pose is extracted from there. They use the libpointmatcher library to match the point clouds.

### APE
The absolute pose error works by taking the output of odometry data, which is a sequence of positions indexed by a timestamp. It is unlikely the the timestamps match, so instead you linearly interpolate the position and spherically interpolate the rotation.


## Point-LIO

## Newer College Dataset

## Point-LIO Results for Newer College Dataset

## Collection 1
nc-quad-easy, 0.068729
nc-quad-medium, 0.057268
nc-quad-hard, 0.057049
stairs, 120.619195 (I can't explain this yet)

