"""Everything pose graph related, adding nodes/edges, and checking for loop closure."""

import numpy as np
import math
from utils import Node, Edge, Pose

class PoseGraph:
    """Stores the graph and scans over time."""
    def __init__(self):
        self.node_list = []
        self.edge_list = []
        self.threshold = (0.5, 0.5, 30)     # x, y, theta

        # First node
        self.check_node: Node = None

        # Boolean for loop closure
        self.closed_loop = False

    def add_node(self, node: Node):
        """Check time and create node"""
        print("---------- ADDING NODE --------------")
        self.node_list.append(node)

    def add_edge(self, edge: Edge):
        """Create an edge between nodes"""
        print("---------- ADDING EDGE --------------")
        self.edge_list.append(edge)

    def get_pose_graph(self):
        """Return current state of pose graph for visuals"""
        return self.node_list, self.edge_list
    
    def assign_weight(self):
        pass
    
    def check_loop_closure(self, node: Node):
        """
        Check if the current node is within the inital node to force loop closure.
        
        Args:
            node: the most recent node to check against
            
        """

        delta = node.pose - self.check_node.pose

        # Check if the pose is within threshold
        if delta <= self.threshold:

            print("------------- LOOP CLOSURE FOUND --------------")

            # Relative transformation matrix
            transform = np.array([
                [math.cos(delta.theta), -math.sin(delta.theta), delta.x],
                [math.sin(delta.theta), math.cos(delta.theta), delta.y],
                [0, 0, 1]])
        
            # Add edge, force loop closure
            edge = Edge(0, node.node_id, transform)

            self.add_edge(edge)

            self.closed_loop = True

    
    def is_loop_closed(self):
        """True or False."""
        return self.closed_loop

        



    