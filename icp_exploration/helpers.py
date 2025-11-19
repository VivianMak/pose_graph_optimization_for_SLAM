import numpy as np
from sklearn.neighbors import KDTree

def pose_to_transform(x, y, theta):
    """Convert (x, y, theta) into a 3×3 homogeneous transform."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

def inverse_transform(T):
    """Inverse of a 2D homogeneous transform."""
    R = T[:2, :2]
    t = T[:2, 2]
    T_inv = np.eye(3)
    T_inv[:2, :2] = R.T
    T_inv[:2, 2] = -R.T @ t
    return T_inv

def transform_between_poses(poseA, poseB):
    """
    Return the homogeneous transform that maps coordinates in frame A
    into coordinates in frame B.
    """
    x1, y1, th1 = poseA
    x2, y2, th2 = poseB

    T_A = pose_to_transform(x1, y1, th1)
    T_B = pose_to_transform(x2, y2, th2)

    return inverse_transform(T_A) @ T_B

def rot2_hom(theta_deg):
    """Return a 3×3 homogeneous rotation matrix for a 2D rotation."""
    theta = np.deg2rad(theta_deg)
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def make_correspondences(src_points, dst_points):
    # Build KD-tree for fast NN search
    tree = KDTree(dst_points.T)

    # Query nearest neighbors
    dists, indices = tree.query(src_points.T, k=1)

    # Make a list of lists containing corresponding indices
    correspondences = [(i, indices[i,0]) for i in range(len(src_points[0]))]
    return correspondences, dists

def svd_rigid_transform(src, dst, dists):
    trans_threshold = 0.5 # Threshold for removing scans that are clearly not visible in dst_points
    trans_mask = (dists < trans_threshold).flatten()

    src = src.T[trans_mask]
    dst = dst.T[trans_mask]

    # Step 1: centroids
    src_centroid = np.mean(src, axis=0)
    dst_centroid = np.mean(dst, axis=0)

    # Step 2: center
    src_centered = src - src_centroid
    dst_centered = dst - dst_centroid

    # Step 3: cross-covariance
    H = src_centered.T @ dst_centered

    # Step 4: SVD
    U, S, Vt = np.linalg.svd(H)

    # Step 5: rotation
    R = Vt.T @ U.T

    # Step 5b: fix reflection
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Step 6: translation
    t = dst_centroid - R @ src_centroid

    return R, t

def htm_2d(R, t):
    """
    Build a 3x3 homogeneous transformation matrix from a 2x2 rotation matrix R
    and a 2-element translation vector t.
    """
    T = np.eye(3)
    T[:2, :2] = R[:2, :2]
    T[:2, 2] = t[:2]
    return T

def iterate_icp(src, dst):
    corresponding_pts, dists = make_correspondences(src, dst)
    corresponding_dst = np.hstack([dst[:, pair[1]].reshape(3, 1) for pair in corresponding_pts])

    rot, trans = svd_rigid_transform(src, corresponding_dst, dists)
    src_to_dst_htm = htm_2d(rot, trans)

    transformed_src = src_to_dst_htm @ src  

    return transformed_src, src_to_dst_htm

def icp(src, dst, num_iterations, odom_htm):
    """
    Parameters:
        src (np.array of size 3, N): x and y values of source lidar scans,
        first row is xs, second row is ys, third row is ones, a column is the coord of a scan.
        dst (np.array of size 3, N): x and y values of destination lidar scans,
        first row is xs, second row is ys, third row is ones, a column is the coord of a scan.
        iterations (int): number of icp iterations
        odom_htm (np.array of size 3, 3): homogeneous transformation matrix from
        pose where the source scan was taken to the pose that the destination scan
        was taken based on odometry.

    Return:
        src_to_dst (np.array of size 3, 3): homogeneous transformation matrix from
        pose where the source scan was taken to the pose that the destination scan
        was taken based on LiDAR.
    """

    src_to_dst = odom_htm
    src_points = odom_htm @ src # doesn't get returned

    for i in range(num_iterations):
        src_points, iteration_htm = iterate_icp(src_points, dst)
        src_to_dst = iteration_htm @ src_to_dst

    return src_to_dst
