import math
import time
 
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