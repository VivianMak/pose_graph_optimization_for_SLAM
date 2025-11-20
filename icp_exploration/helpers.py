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

def compute_normals(dst_points, num_neighbors=6):
    """
    dst_points: (3, N) homogeneous
    returns normals: (2, N)
    """
    pts = dst_points[:2, :].T  # (N,2)
    tree = KDTree(pts)
    N = pts.shape[0]
    normals = np.zeros((2, N))
    for i in range(N):
        k = min(num_neighbors, N)
        _, idxs = tree.query(pts[i].reshape(1,-1), k=k)
        neighbors = pts[idxs[0]]     # (k,2)
        cov = np.cov(neighbors.T)
        # eigenvector of smallest eigenvalue -> normal
        evals, evecs = np.linalg.eigh(cov)
        n = evecs[:, 0]
        normals[:, i] = n

    # Orient normals to point roughly toward centroid (simple heuristic)
    centroid = np.mean(pts, axis=0)
    vec_to_centroid = (centroid - pts).T  # shape (2, N)
    dot = np.sum(normals * vec_to_centroid, axis=0)
    flip_mask = dot > 0   # if normal points toward centroid, flip so it points outward
    normals[:, flip_mask] *= -1

    return normals


def make_correspondences(src_points, dst_points):
    # Build KD-tree for fast NN search
    tree = KDTree(dst_points[:2, :].T)

    # Query nearest neighbors
    dists, indices = tree.query(src_points[:2, :].T, k=1)

    # Make a list of lists containing corresponding indices
    correspondences = [(i, indices[i,0]) for i in range(len(src_points[0]))]
    return correspondences, dists

def least_squares_transform(src, dst, normals, dists, threshold, sigma):
    # Point to plane optimization
    mask = (dists < threshold).flatten()
    src = src[:2, mask].T
    dst = dst[:2, mask].T
    normals = normals[:, mask].T

    A = np.zeros((len(src), 3)) # Transformation to move src_point along normal
    b = np.zeros(len(src))  # How far p is from the tangent along the normal
    weights = np.zeros(len(src))

    for i in range(len(src)):
        src_point = src[i]
        dst_point = dst[i]
        normal_vector = normals[i]
        error = normal_vector @ (src_point - dst_point)        # point-to-plane, @ is a dot product with two vectors 
        weights[i] = np.exp(-(error**2) / (2*sigma**2))  # Gaussian weighting
        A[i] = [-src_point[1]*normal_vector[0] + src_point[0]*normal_vector[1], normal_vector[0], normal_vector[1]]
        b[i] = normal_vector @ (dst_point - src_point)

    # Apply weights
    W_sqrt = np.sqrt(weights)
    A_weighted = A * W_sqrt[:, np.newaxis]
    b_weighted = b * W_sqrt

    # Solve weighted least squares
    x, _, _, _ = np.linalg.lstsq(A_weighted, b_weighted, rcond=None)
    theta, tx, ty = x
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    t = np.array([tx, ty])
    return htm_2d(R, t)


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
    T[:2, :2] = R
    T[:2, 2] = t
    return T

def iterate_icp(src, dst, normals):
    corresponding_pts, dists = make_correspondences(src, dst)
    corresponding_dst = np.hstack([dst[:, pair[1]].reshape(3, 1) for pair in corresponding_pts])
    corresponding_norms = normals[:, [pair[1] for pair in corresponding_pts]]

    error_threshold = 1
    error_weighting = 0.5

    src_to_dst_htm = least_squares_transform(
        src,
        corresponding_dst,
        corresponding_norms,
        dists,
        error_threshold,
        error_weighting
    )
    transformed_src = src_to_dst_htm @ src

    return transformed_src, src_to_dst_htm

def icp(src, dst, num_iterations, odom_htm, num_neighbors):
    """
    Parameters:
        src (np.array of size 3, N): x and y values of source lidar scans,
        first row is xs, second row is ys, third row is ones, a column is the coord of a scan.
        dst (np.array of size 3, N): x and y values of destination lidar scans,
        first row is xs, second row is ys, third row is ones, a column is the coord of a scan.
        num_iterations (int): number of icp iterations
        odom_htm (np.array of size 3, 3): homogeneous transformation matrix from
        pose where the source scan was taken to the pose that the destination scan
        was taken based on odometry.

    Return:
        src_to_dst (np.array of size 3, 3): homogeneous transformation matrix from
        pose where the source scan was taken to the pose that the destination scan
        was taken based on LiDAR.
    """

    normals = compute_normals(dst, num_neighbors)
    src_to_dst = odom_htm
    src_points = odom_htm @ src # doesn't get returned

    for i in range(num_iterations):
        src_points, iteration_htm = iterate_icp(src_points, dst, normals)
        src_to_dst = iteration_htm @ src_to_dst

    return src_to_dst
