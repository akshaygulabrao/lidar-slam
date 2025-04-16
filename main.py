import numpy as np
import os
from pykitti import raw
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.transform import Rotation as R
import time
from tqdm import tqdm

# Dataset loading (your existing code)
basedir = '/home/ox/Kitti-Dataset'
date = '2011_09_26'
drive = '0001'
dataset = raw(basedir, date, drive)

def compute_ate(estimated_trajectory, ground_truth_trajectory):
    """
    Compute Absolute Trajectory Error (ATE) between estimated and ground truth trajectories
    
    Args:
        estimated_trajectory: List of 4x4 transformation matrices
        ground_truth_trajectory: List of 4x4 transformation matrices
        
    Returns:
        ate: Absolute Trajectory Error (RMSE)
        errors: List of individual errors at each timestep
    """
    # Align trajectories using first pose
    first_est = estimated_trajectory[0]
    first_gt = ground_truth_trajectory[0]
    
    # Compute alignment transformation
    alignment = first_gt @ np.linalg.inv(first_est)
    
    # Align estimated trajectory
    aligned_estimated = [alignment @ pose for pose in estimated_trajectory]
    
    # Extract positions
    est_positions = np.array([pose[:3, 3] for pose in aligned_estimated])
    gt_positions = np.array([pose[:3, 3] for pose in ground_truth_trajectory])
    
    # Calculate errors
    errors = np.linalg.norm(est_positions - gt_positions, axis=1)
    ate = np.sqrt(np.mean(errors**2))
    
    return ate, errors

def get_ground_truth_trajectory(dataset, num_frames):
    """Extract ground truth trajectory from KITTI dataset"""
    gt_trajectory = []
    
    # KITTI poses are given as 4x4 transformation matrices
    for i in range(num_frames):
        # Get pose from OXTS data (ground truth)
        oxts = dataset.oxts[i]
        pose = oxts.T_w_imu  # Transformation from IMU to world
        
        gt_trajectory.append(pose)
    
    return gt_trajectory

def preprocess_point_cloud(points, max_points=50000):
    """Downsample and remove outliers from point cloud"""
    # Remove points with zero reflectance and invalid coordinates
    mask = (points[:, 0] != 0) & (points[:, 1] != 0) & (points[:, 2] != 0)
    points = points[mask]
    
    # Simple random downsampling if too many points
    if len(points) > max_points:
        indices = np.random.choice(len(points), max_points, replace=False)
        points = points[indices]
    
    return points

def icp(source, target, max_iterations=20, tolerance=1e-3):
    """Basic Iterative Closest Point algorithm"""
    prev_error = 0
    
    # Initial transformation (identity)
    transformation = np.eye(4)
    
    for i in range(max_iterations):
        # Find nearest neighbors
        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target[:, :3])
        distances, indices = nbrs.kneighbors(source[:, :3])
        
        # Compute current error
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
        
        # Compute transformation between source and target
        H = source[:, :3].T @ target[indices[:, 0], :3]
        U, S, Vt = np.linalg.svd(H)
        rotation = Vt.T @ U.T
        
        # Handle reflection case
        if np.linalg.det(rotation) < 0:
            Vt[-1, :] *= -1
            rotation = Vt.T @ U.T
            
        translation = target[indices[:, 0], :3].mean(axis=0) - (rotation @ source[:, :3].T).T.mean(axis=0)
        
        # Update transformation
        current_transformation = np.eye(4)
        current_transformation[:3, :3] = rotation
        current_transformation[:3, 3] = translation
        transformation = current_transformation @ transformation
        
        # Update source points
        source = (rotation @ source[:, :3].T).T + translation
    
    return transformation

def benchmark_slam(dataset, num_frames=100):
    """Benchmark a simple SLAM implementation with ATE calculation"""
    timings = []
    trajectory = [np.eye(4)]  # Start with identity transformation
    
    # Get ground truth trajectory
    gt_trajectory = get_ground_truth_trajectory(dataset, num_frames)
    
    # Pre-load all point clouds
    point_clouds = []
    for i in tqdm(range(num_frames), desc="Loading point clouds"):
        velo = dataset.get_velo(i)
        pc = preprocess_point_cloud(velo)
        point_clouds.append(pc)
    
    for i in tqdm(range(1, num_frames), desc="Processing frames"):
        start_time = time.time()
        
        # Get consecutive point clouds
        source = point_clouds[i-1]
        target = point_clouds[i]
        
        # Estimate transformation using ICP
        transformation = icp(source, target)
        
        # Update global trajectory
        global_pose = transformation @ trajectory[-1]
        trajectory.append(global_pose)
        
        # Record timing
        timings.append(time.time() - start_time)
    
    # Calculate metrics
    avg_time = np.mean(timings)
    fps = 1 / avg_time if avg_time > 0 else 0
    
    # Compute ATE
    ate, errors = compute_ate(trajectory, gt_trajectory)
    
    print(f"\nBenchmark Results:")
    print(f"- Average processing time per frame: {avg_time:.4f} seconds")
    print(f"- Average FPS: {fps:.2f}")
    print(f"- Absolute Trajectory Error (ATE): {ate:.4f} meters")
    
    return trajectory, timings, ate, errors, gt_trajectory

trajectory, timings, ate, errors, gt_trajectory = benchmark_slam(dataset, num_frames=100)

def plot_trajectory_comparison(estimated_trajectory, gt_trajectory):
    """Plot both estimated and ground truth trajectories"""
    est_positions = np.array([pose[:3, 3] for pose in estimated_trajectory])
    gt_positions = np.array([pose[:3, 3] for pose in gt_trajectory])
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(est_positions[:, 0], est_positions[:, 1], est_positions[:, 2], 
            'b-', label='Estimated Trajectory')
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2], 
            'r-', label='Ground Truth')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Trajectory Comparison')
    ax.legend()
    plt.show()

plot_trajectory_comparison(trajectory, gt_trajectory)