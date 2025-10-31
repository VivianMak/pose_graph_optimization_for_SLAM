from utils import Node, Edge

class PoseGraph:
    """Stores the graph and scans over time."""
    def __init__(self):
        self.node_list = []
        self.edge_list = []

    def create_node(self, id, pose, scans):
        """Check time and create node"""
        self.node_list.append(Node(id, pose, scans))

    def create_edge(self, mat, parent_id, child_id):
        """Create an edge between nodes"""
        self.edge_list.append(Edge(mat, parent_id, child_id))

    def get_pose_graph(self):
        """Return current state of pose graph for visuals"""
        return self.node_list, self.edge_list
    
    def check_scans(self):
        """Check if a scan is aligned with some previous,
        if so, create edge"""
        pass


    