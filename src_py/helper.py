import math
import time
import numpy as np

from utils import Point
 
def theta_from_quaternion(z, w):
    """
        Convert a quaternion into theta.
        Yaw is rotation around z in radians (counterclockwise).

        Args:
        - z: from odom msg
        - w: from odom msg

    """
    theta = math.atan2(2*(w*z), 1-(2*(z**2)))
    return theta
    
def wrap(theta):
    """Wrap angle to [-pi, pi)."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


class Timer:
    def __init__(self):
        self.start = time.monotonic()  # like std::chrono::steady_clock::now()
        print("-------- STARTING TIMER ----------")

    def elapsed(self):
        return time.monotonic() - self.start
    

def polar_to_cartesian(scan):
    """Converts a laser scan from polar to cartesian, keeping index.
    Returns:
        pts: a list of Points(x, y, index, r)"""
    pts = []
    for i, r in enumerate(scan):
        if r <= 0:   # invalid reading
            pts.append(None)
            continue
        angle = math.radians(i)
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        pts.append(Point(x, y, i, r))
    return pts


def transform_scan(T, coords):
    """Apply a 3x3 homogeneous transformation on list[(x,y)]"""

    new_coords = []
    r_trans = []
    
    for x, y in coords:

        if x is None or y is None:
            new_coords.append(None)
            r_trans.append(None)
            continue

        x_t, y_t, _ = T @ np.array([x, y, 1.0])
        new_coords.append((x_t, y_t))
        r_trans.append(math.hypot(x_t, y_t))

    return new_coords, r_trans
