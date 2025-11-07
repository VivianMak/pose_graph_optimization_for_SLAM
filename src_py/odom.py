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
        self.transform = np.eye(3)


    def update(self, msg):
        """
        Updates the odometry based on sim data.
        
        Args:
            msg: a dictionary of time and pose data"""
        
        print("---------- COMPUTING NEW POSE --------------")
        self.parse_pose(msg)
        self.compute_new_pose()
        
        # Assign current pose to old pose
        self.prev_pose = self.current_pose
        

    def compute_new_pose(self):
        """Find new pose and compute relative tranform matrix once time elapsed."""
        
        # By setting prev_pose to (000), we handle the inital case elsewhere

        # Pose delta
        delta = self.current_pose - self.prev_pose

        # Relative transformation matrix
        self.transform = np.array([
            [math.cos(delta.theta), -math.sin(delta.theta), delta.x],
            [math.sin(delta.theta), math.cos(delta.theta), delta.y],
            [0, 0, 1]])
        

    def parse_pose(self, msg):
        """Converts a message into a pose."""
        theta = theta_from_quaternion(
            msg["orientation_z"],
            msg["orientation_w"]
        )

        self.current_pose = Pose(msg["position_x"], msg["position_y"], theta)

    
    def get_scans(self, msg):
        """
        Reads lidar data from message.
        Args:
            msg: a dictionary of time and scan data"""
        
        return msg["ranges"]

    def get_current_pose(self) -> Pose:
        """Return current timestep's pose (x,y,theta)"""
        return self.current_pose
    
    def get_transform(self):
        """Returns transformation matrix."""
        return self.transform


    
    

