import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from helpers import transform_between_poses, icp

# Load scan and odom data
scan_data = np.loadtxt("lidar_scans.csv", delimiter=",")

noisy_data = pd.read_csv("noisy_odom_data.csv")

# Get odom values
noisy_xs = noisy_data["x"].values
noisy_ys = noisy_data["y"].values
noisy_thetas = noisy_data["theta"].values

# Choose indices
pose_id_one = 0
pose_id_two = 2695

# pose_id_one = 500
# pose_id_two = 620

# Select chosen scans
scan = scan_data[pose_id_one]   # first scan
scan = np.where(np.isfinite(scan), scan, 0.0)
angles = np.linspace(0, 2*np.pi, len(scan), endpoint=False)
xs = scan * np.cos(angles)
ys = scan * np.sin(angles)

scan2 = scan_data[pose_id_two]   # second scan
scan2 = np.where(np.isfinite(scan2), scan2, 0.0)
angles2 = np.linspace(0, 2*np.pi, len(scan2), endpoint=False)
xs2 = scan2 * np.cos(angles2)
ys2 = scan2 * np.sin(angles2)

# Make array to transform seconds set of scans
scan2_array = np.array([
    xs2,
    ys2,
    np.ones(len(scan2))
])

# Points
point_xs = [noisy_xs[pose_id_one], noisy_xs[pose_id_two]]
point_ys = [noisy_ys[pose_id_one], noisy_ys[pose_id_two]]

# Arrow directions
dir_xs = [np.cos(noisy_thetas[pose_id_one]), np.cos(noisy_thetas[pose_id_two])]
dir_ys = [np.sin(noisy_thetas[pose_id_one]), np.sin(noisy_thetas[pose_id_two])]

htm_one_two = transform_between_poses(
    [noisy_xs[pose_id_one], noisy_ys[pose_id_one], noisy_thetas[pose_id_one]],
    [noisy_xs[pose_id_two], noisy_ys[pose_id_two], noisy_thetas[pose_id_two]]
)

src_points = np.vstack((scan2_array[0], scan2_array[1], scan2_array[2]))
dst_points = np.vstack((xs, ys, np.ones(len(xs)))) # size (3, 640)

odom_transform_src = htm_one_two @ src_points

num_iterations = 100
src_to_dst = icp(src_points, dst_points, num_iterations, htm_one_two)

transformed_src = src_to_dst @ src_points

plt.quiver(
    point_xs,
    point_ys, 
    dir_xs,
    dir_ys, angles='xy', scale_units='xy', scale=5
)
plt.scatter(point_xs, point_ys, color='red')
plt.axis("equal")
plt.scatter(xs, ys, s=2, c='blue')  # s=point size
plt.scatter(src_points[0, :], src_points[1, :], s=2, c='red')
plt.scatter(odom_transform_src[0, :], odom_transform_src[1, :], s=2, c='orange')
plt.scatter(transformed_src[0, :], transformed_src[1, :], s=2, c='green')
plt.plot(noisy_xs, noisy_ys)
plt.show()