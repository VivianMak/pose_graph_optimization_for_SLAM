import numpy as np
from sklearn.neighbors import KDTree
import math

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
    tree = KDTree(dst_points)

    # Query nearest neighbors
    dists, indices = tree.query(src_points, k=1)

    # (i, j) pairs
    correspondences = [(i, indices[i,0]) for i in range(len(src_points))]
    return correspondences

def svd_rigid_transform(src, dst):
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
        Vt[2, :] *= -1
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
    T[:2, :2] = R
    T[:2, 2] = t
    return T