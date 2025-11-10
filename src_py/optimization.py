import math
from scipy.spatial import cKDTree

from helper import Timer
from odom import Odometry
from pose_graph import PoseGraph

from utils import Node, Edge, Pose
from helper import polar_to_cartesian, transform_scan

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

        print("------- ROBOT MOVING --------")

        # Run loop until closure
        while self.graph.is_loop_closed is False:

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

                # With two or more nodes
                if self.id > 0:
                    transform = self.odom.get_transform()
                    # Add edge
                    edge = Edge(self.id-1, self.id, transform)
                    self.graph.add_edge(edge)

                    # Check if there is a loop closure
                    self.graph.check_loop_closure(node)
                else:
                    # On first timestep, save node to check threshold
                    self.graph.check_node = node

                # Increment node id
                self.id += 1

            nodes, edges = self.graph.get_pose_graph()
            print(f"NODES ARE: {nodes}")
            print(f"EDGES ARE: {edges}")

        # Loop closure - start going backwards to optimize




    def optimize(self):
        """to be implemented..."""
        pass



    def compare_scans(self, edge: Edge, node1: Node, node2: Node):
        """
        Compare lidar scans from node1 and node2 using KD-tree nearest-neighbor geometric matching. This evaluates confidence metric on the pose transform based on the overlap between the two scans.
        
        Args:
            edge: the Edge between the two nodes
            node1: Node to assign weight to
            node2: Previous node to compare scans to

        Return:
            confidence
            
        """
        # TODO: set invalid scans to 0

        # Set variables
        residuals = []
        matches = 0

        # Tunable thresholds
        MAX_NN_DIST      = 0.25   # max allowed nearest-neighbor distance (m)
        OCCLUSION_MARGIN = 0.05   # (m)
        MAX_ERROR        = 1.00   # normalization scale for RMSE

        # Convert scans from polar to cartesian
        scan1 = polar_to_cartesian(node1.scans)     # list[Point]
        scan2 = polar_to_cartesian(node2.scans)     # list[Point]

        # Extract coodinates from Point dataclass
        coords1 = [(p.x, p.y) for p in scan1 if p is not None]
        coords2 = [(p.x, p.y) for p in scan2 if p is not None]

        # Match order of coords 1
        ordered_coords1 = [p for p in scan1 if p is not None]

        # Initialize query tree
        tree = cKDTree(coords1)

        # Apply transformation on scan2
        transform = edge.transform
        coords2, r_trans = transform_scan(transform, coords2)

        for point, r in zip(coords2, r_trans):
            
            if point is None:
                continue

            dist, idx = tree.query(point, k=1)

            if dist > MAX_NN_DIST:
                continue

            # Get matched point
            matched_pt1 = ordered_coords1[idx]

            # ============================
            # Occlusion Test
            # ============================
            # If scan2 hit something significantly closer than the transformed pt,
            # the transformed pt is hidden behind an obstacle → skip.
            if matched_pt1.r < r_trans - OCCLUSION_MARGIN:
                continue

            # ============================
            # Accept match
            # ============================
            residuals.append(dist)
            matches += 1

        # Convert matches and residuals to a confidence score
        if matches == 0:
            return 0.0
        
        rmse = math.sqrt(sum(r*r for r in residuals) / matches)
        overlap_ratio = matches / len(scan1)

        # Normalize RMSE into [0,1]
        rmse_norm = max(0.0, 1.0 - rmse / MAX_ERROR)
        confidence = overlap_ratio * rmse_norm

        return confidence

    

