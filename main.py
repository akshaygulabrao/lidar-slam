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
    """Benchmark a simple SLAM implementation"""
    timings = []
    trajectory = [np.eye(4)]  # Start with identity transformation
    
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
    
    print(f"\nBenchmark Results:")
    print(f"- Average processing time per frame: {avg_time:.4f} seconds")
    print(f"- Average FPS: {fps:.2f}")
    
    return trajectory, timings

# Run benchmark on first 100 frames
trajectory, timings = benchmark_slam(dataset, num_frames=100)

# Visualize trajectory
def plot_trajectory(trajectory):
    positions = [pose[:3, 3] for pose in trajectory]
    positions = np.array(positions)
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', label='Estimated Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('LIDAR SLAM Trajectory')
    ax.legend()
    plt.show()

plot_trajectory(trajectory)