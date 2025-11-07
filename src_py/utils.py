import numpy as np

from dataclasses import dataclass
from helper import wrap

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

    def __sub__(self, old):
        return Pose(
            x = self.x - old.x,
            y = self.y - old.y,
            theta = wrap(self.theta - old.theta),
        )

@dataclass
class Node:
    node_id: int
    pose: Pose      # uncertain
    scans: None     # laser scans will be in a list


    def to_dict(self):
        return {
            "id": self.node_id,
            "pose": self.pose,
            "scans": self.scans,
        }
    
@dataclass
class Edge:
    parent_id: int
    child_id: int
    transform: np.ndarray   # 3x3 homogenous tranform matrix
    # info: np.ndarray        # 3x3 information matrix (inverse covariance)

