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

def distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def make_correspondences(src_points, dst_points):
    # Build KD-tree for fast NN search
    tree = KDTree(dst_points)

    # Query nearest neighbors
    dists, indices = tree.query(src_points, k=1)

    # (i, j) pairs
    correspondences = [(i, indices[i,0]) for i in range(len(src_points))]
    return correspondences