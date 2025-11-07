"""Reads the rosbag file to parse all relevant data and compute Node/Edge info."""
import numpy as np
import math

from helper import theta_from_quaternion
from utils import Pose

class Odometry:
    """"""
    def __init__(self):
        """Initialize isntance of odom class."""

        self.current_pose: Pose = Pose(0,0,0)
        self.prev_pose: Pose = Pose(0,0,0)   # initalizing odom frame pose
        self.trans_mat = np.eye(3)

        self.current_time = 0
        self.prev_time = 0
        self.time_threshold = 2 #sec
        self.time_passed = False
        

    def compute_new_pose(self) -> Pose:
        """Find new pose and compute relative tranform matrix once time elapsed."""
        
        # Check if enough time has passed
        if self.time_passed:
            self.get_odom()

        # Find change in pose
        delta = self.current_pose - self.prev_pose

        # Compute transformation matrix
        self.trans_mat = np.array([math.cos(delta.theta), -math.sin(delta.theta), delta.x],
                                  [math.sin(delta.theta), math.cos(delta.theta), delta.y],
                                  [0, 0, 1])
               
        # Assign current pose to old pose
        self.prev_pose = self.current_pose

        # Reset time_passed to pase
        self.time_passed = False

        # Returns pose and scans
        self.populate_node()
    
    def get_time(self):
        """
        Continuously run the timer.
        Returns time passed.
        """
        
        # TODO: Continously read/subscribe to header msg (timer) of the rosbag

        # Find elapsed time since prev
        elapsed = self.timer - self.prev_time

        # After the threshold time has passed
        if elapsed >= self.time_threshold:
            self.time_passed = True
            self.prev_time = self.current_time
        
        return self.time_passed


    def get_odom(self):
        """
        Reads wheel odom from bag file at current time.
        Returns new odom """

        # convert quaternion to Pose dataclass
            # position:
            #     x: -1.5715268228573835
            #     y: -0.852135447663224
            #     z: 0.0
            # orientation:
            #     x: 0.0
            #     y: 0.0
            #     z: -0.9941510062826789
            #     w: 0.10799896623179726

        # Example from above
        position = [-1.5715268228573835, -0.852135447663224, 0]       # x, y, z
        orientation = [0, 0, -0.9941510062826789, 0.10799896623179726]    # x, y, z, w

        x = position[0]
        y = position[1]

        theta = theta_from_quaternion(orientation[2], orientation[3])
        self.current_pose = Pose(x, y, theta)
    
    def get_scans(self):
        """Get the laser scan at x timestep."""
        scans = []
        self.current_time
        # parse data from rosbag
        return scans

    def get_new_pose(self) -> Pose:
        """Return current timestep's pose (x,y,theta)"""
        return self.current_pose
    
    def get_trans_mat(self):
        """Returns transformation matrix."""
        return self.trans_mat
    
    def populate_node(self):
        """Return the scan and pose info to make a node"""
        scans = self.get_scans()
        return self.current_pose, scans

    
    

