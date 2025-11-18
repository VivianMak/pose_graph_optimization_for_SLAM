import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from helpers import transform_between_poses, make_correspondences, svd_rigid_transform, htm_2d

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

transformed_scan2 = htm_one_two @ scan2_array
# print(transformed_scan2)

src_points = np.vstack((transformed_scan2[0], transformed_scan2[1])).T
dst_points = np.vstack((xs, ys)).T

correspondences = make_correspondences(src_points, dst_points) # source points are the current lidar scans, destination is what we're comparing to

corresponding_dst = np.vstack([dst_points[[pair[1]]] for pair in correspondences])

rot, trans = svd_rigid_transform(src_points, corresponding_dst)

src_to_dst_htm = htm_2d(rot, trans)

transformed_src = src_to_dst_htm @ transformed_scan2

plt.quiver(
    point_xs,
    point_ys, 
    dir_xs,
    dir_ys, angles='xy', scale_units='xy', scale=5
)
plt.scatter(point_xs, point_ys, color='red')
plt.axis("equal")
plt.scatter(xs, ys, s=5, c='blue')  # s=point size
# plt.scatter(xs2, ys2, s=5, c='green')
plt.scatter(transformed_scan2[0], transformed_scan2[1], s=5, c='red')
plt.scatter(transformed_src[0], transformed_src[1], s=5, c='green') 
plt.plot(noisy_xs, noisy_ys)
    
for i in range(len(correspondences)):
    plt.plot([src_points[i,0], corresponding_dst[i,0]],
             [src_points[i,1], corresponding_dst[i,1]],
             c='gray', linewidth=0.5)

# plt.xlim([0.0, 3.0])
# plt.ylim([-2.0, 1.0])
plt.show()