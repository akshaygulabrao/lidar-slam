import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# Define file path
fname = "/home/ox/lidar-slam/collection_1/ground_truth/gt-nc-quad-easy.csv"

# Read the CSV file with column names
column_names = ['sec', 'nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
try:
    df = pd.read_csv(fname, comment='#', names=column_names)
except FileNotFoundError:
    print(f"Error: File not found at {fname}")
    exit()

# Create 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the XYZ coordinates as a line
ax.plot(df['x'], df['y'], df['z'], 'b-', linewidth=1.5, alpha=0.8)

# Add starting and ending markers
ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], 
           c='green', marker='o', s=100, label='Start')
ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], 
           c='red', marker='x', s=100, label='End')

# Set labels and title
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Z Position (m)')
ax.set_title('3D Trajectory: gt-nc-quad-easy.csv')

# Add grid and legend
ax.grid(True)
ax.legend()

# Adjust viewing angle for better perspective
ax.view_init(elev=20, azim=45)

# Show the plot
plt.tight_layout()
plt.show()
