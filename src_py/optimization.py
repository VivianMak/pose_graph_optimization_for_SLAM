import math
from helper import Timer
from odom import Odometry
from pose_graph import PoseGraph

from utils import Node, Edge, Pose

FAKE_SCAN = [
    {
        "ranges": [1.0, 1.2, 1.1, 0.9, 0.8, 0.85, 1.05, 1.3, 1.1, 0.95],
        "sec": 1,
        "nanosec": 0
    },
    {
        "ranges": [1.1, 1.3, 1.0, 0.95, 0.75, 0.9, 1.0, 1.2, 1.15, 0.98],
        "sec": 2,
        "nanosec": 500000000
    },
    {
        "ranges": [0.9, 1.0, 1.2, 1.3, 1.1, 1.0, 0.95, 1.05, 0.9, 0.85],
        "sec": 3,
        "nanosec": 250000000
    }
]

FAKE_POSE = [
    {
        "sec": 0, "nanosec": 0,
        "position_x": 0.0, "position_y": 0.0, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0, "orientation_z": 0.0, "orientation_w": 1.0,
        "linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0,
    },
    {
        "sec": 1, "nanosec": 0,
        "position_x": 1.0, "position_y": 0.0, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0, "orientation_z": 0.0, "orientation_w": 1.0,
        "linear_x": 1.0, "linear_y": 0.0, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0,
    },
    {
        "sec": 2, "nanosec": 0,
        "position_x": 2.0, "position_y": 0.1, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0, 
        "orientation_z": math.sin(0.05/2), "orientation_w": math.cos(0.05/2),
        "linear_x": 1.0, "linear_y": 0.1, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.05,
    },
    {
        "sec": 3, "nanosec": 0,
        "position_x": 3.0, "position_y": 0.2, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin(0.10/2), "orientation_w": math.cos(0.10/2),
        "linear_x": 1.0, "linear_y": 0.1, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.05,
    },
    {
        "sec": 4, "nanosec": 0,
        "position_x": 3.1, "position_y": 1.0, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin((math.pi/2)/2), "orientation_w": math.cos((math.pi/2)/2),
        "linear_x": 0.1, "linear_y": 0.8, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": math.pi/2,
    },
    {
        "sec": 5, "nanosec": 0,
        "position_x": 3.0, "position_y": 2.0, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin((math.pi/2 + 0.05)/2), "orientation_w": math.cos((math.pi/2 + 0.05)/2),
        "linear_x": -0.1, "linear_y": 1.0, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.05,
    },
    {
        "sec": 6, "nanosec": 0,
        "position_x": 2.0, "position_y": 3.0, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin((math.pi)/2), "orientation_w": math.cos((math.pi)/2),
        "linear_x": -1.0, "linear_y": 1.0, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": math.pi/2,
    },
    {
        "sec": 7, "nanosec": 0,
        "position_x": 1.0, "position_y": 3.1, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin((math.pi + 0.05)/2), "orientation_w": math.cos((math.pi + 0.05)/2),
        "linear_x": -1.0, "linear_y": 0.1, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.05,
    },
    {
        "sec": 8, "nanosec": 0,
        "position_x": 0.1, "position_y": 2.9, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin((-math.pi/2)/2), "orientation_w": math.cos((-math.pi/2)/2),
        "linear_x": -0.9, "linear_y": -0.2, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": -math.pi/2,
    },
    {
        "sec": 9, "nanosec": 0,
        "position_x": 0.05, "position_y": 0.05, "position_z": 0.0,
        "orientation_x": 0.0, "orientation_y": 0.0,
        "orientation_z": math.sin(-0.05/2), "orientation_w": math.cos(-0.05/2),   # near original orientation
        "linear_x": -0.05, "linear_y": -2.85, "linear_z": 0.0,
        "angular_x": 0.0, "angular_y": 0.0, "angular_z": -0.05,
    }
]



class Optimization:
    """Checks for overlapping scans"""
    def __init__(self):

        # Initialize instances of the class
        self.odom = Odometry()
        self.graph = PoseGraph()

        # Define some hueristic to quantify graph
        self.graph_threshold = 1

        self.id = 0

        # Global timer
        self.timer = Timer()       # Continously runs timer
        # self.current_time = 0
        self.prev_time = None
        self.time_threshold = 1    # secs


    def run_loop(self):
        """Run until opimization graph threshold"""

        print("------- RUNNNIG OPTIMIZATION --------")

        for i in range(10):
            
            # Time condition
            # if self.timer.elapsed() >= self.time_threshold:
                
            pose_msg = FAKE_POSE[i]
            scan_msg = [0,1,2,3]
            self.odom.update(pose_msg)

            # Extract node/edge data 
            pose = self.odom.get_current_pose()
            transform = self.odom.get_transform()

            # Add node
            node = Node(self.id, pose, scan_msg)
            self.graph.add_node(node)

            # Check first timestep
            if self.id > 0:
                transform = self.odom.get_transform()
                # Add edge
                edge = Edge(self.id-1, self.id, transform)
                self.graph.add_edge(edge)

            # Increment node id
            self.id += 1

        nodes, edges = self.graph.get_pose_graph()
        print(f"NODES ARE: {nodes}")
        print(f"EDGES ARE: {edges}")
    

