from odom import Odometry
from pose_graph import PoseGraph


class Optimization:
    """Checks for overlapping scans"""
    def __init__(self):

        # Initialize instances of the class
        self.odom = Odometry
        self.graph = PoseGraph

        # Define some hueristic to quantify graph
        self.graph_threshold = 1

        self.id = 0


    def run_loop(self):
        """Run until opimization graph threshold"""

        while self.graph_threshold > 2:
            if self.odom.get_time:
                pose = self.odom.get_new_pose
                mat = self.odom.get_trans_mat
                scans = self.odom.get_scans

                self.graph.create_node(self.id, pose, scans)

                # Check if it is the first time stamp
                if self.id == 0:
                    pass
                else:
                    self.graph.create_edge(mat, self.id-1, self.id)

            # New node id
            self.id =+ 1

