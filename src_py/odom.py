""""""

from utils import Pose

class Odometry:
    """"""
    def __init__(self):
        """Initialize isntance of odom class."""

        self.pose: Pose = (0,0,0)
        self.old_pose = None
        self.trans_mat = None

        self.timer = 0
        self.time_threshold = 2 #sec
        self.time_passed = False

    def compute_new_pose(self) -> Pose:
        """Find new pose and compute relative? tranform matrix."""
        
        # Account for first timestep

        # Check if enough time has passed
        if self.get_time:
            self.get_odom
        
        # Compute relative transformation matrix -- to apply on the laserscans later
            # Or from global frame
               
    
        self.old_pose = self.pose
        self.time_passed = False
    
    def get_time(self):
        """
        Continuously run the timer.
        Returns time passed."""
        # if x time has passed set to true
        return self.time_passed


    def get_odom(self):
        """
        Reads wheel odom from bag file at current time.
        Returns new odom """

        # convert quaternion to euler
            # position:
            #     x: -1.5715268228573835
            #     y: -0.852135447663224
            #     z: 0.0
            # orientation:
            #     x: 0.0
            #     y: 0.0
            #     z: -0.9941510062826789
            #     w: 0.10799896623179726

        
        self.pose = Pose()

    def get_new_pose(self) -> Pose:
        return self.pose
    
    def get_trans_mat(self):
        return self.trans_mat
    
    def get_scans(self):
        """Get the laser scan at x timestep."""
        scans = []
        # parse data from rosbag
        return scans

