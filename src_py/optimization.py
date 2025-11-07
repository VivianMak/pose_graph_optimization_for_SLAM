from odom import Odometry
from pose_graph import PoseGraph

from utils import Node, Edge, Pose


class Optimization:
    """Checks for overlapping scans"""
    def __init__(self):

        # Initialize instances of the class
        self.odom = Odometry()
        self.graph = PoseGraph()

        # Define some hueristic to quantify graph
        self.graph_threshold = 1

        self.id = 0


    def run_loop(self):
        """Run until opimization graph threshold"""

        while self.graph_threshold > 2:
            if self.odom.get_time():
                pose = self.odom.get_new_pose()
                
                scans = self.odom.get_scans()
                node = Node(self.id, pose, scans)
                self.graph.add_node(node)

                self.graph.add_node(self.id, pose, scans)

                # Check if it is the first time stamp
                if self.id > 0:
                    transform = self.odom.get_trans_mat()
                    edge = Edge(self.id-1, self.id, transform)
                    self.graph.add_edge(edge)

            # Increment node id
            self.id += 1

