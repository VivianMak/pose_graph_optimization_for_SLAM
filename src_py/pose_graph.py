from utils import Node, Edge

class PoseGraph:
    """Stores the graph and scans over time."""
    def __init__(self):
        self.node_list = []
        self.edge_list = []
        self.threshold = (0.2, 0.2, 30)     # x, y, theta

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
    
    def check_loop_closure(self):
        """Check if a scan is aligned with some previous,
        if so, create edge"""
        pass


    