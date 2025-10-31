from dataclasses import dataclass

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float


@dataclass
class Node:
    node_id: int
    pose: Pose
    scans: list     # laser scans will be in a list


    def to_dict(self):
        return {
            "id": self.node_id,
            "pose": self.pose,
            "scans": self.scans,
        }
    
@dataclass
class Edge:
    transform = [] # turn into np array
    parent_id = int
    child_id = int