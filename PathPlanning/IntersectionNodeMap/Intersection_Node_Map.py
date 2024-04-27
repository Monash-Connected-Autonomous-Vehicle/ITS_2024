import sys
import pathlib
import yaml
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

sys.path.append(str(pathlib.Path(__file__).parent.parent))
from MRNEnv.environment import Env, RasterEnv

from dataclasses import dataclass
from abc import ABC, abstractmethod

@dataclass(frozen=True) # must be frozen to inherit hashing
class IntersectionNode:
    """ 
    A single node in a node map.
    
    name: Alias for the node to be referenced in yaml files
    xpos, ypos: coordinates of the node in the environment [m]
    leftNode, rightNode, aheadNode, uturnNode: Other node names which dictate the digraph structure
    """
    name: str = None
    xPos: float = 0
    yPos: float = 0
    leftNode: str = None
    rightNode: str = None
    aheadNode: str = None
    uturnNode: str = None

@dataclass
class NodeMap(ABC):
    """
    The node map for a given environment. The map is internally represented as a digraph. 

    DG: The internal digraph
    env: Underlying environment object for the node map
    """
    DG: nx.DiGraph = nx.DiGraph()
    env: Env = None

    def from_yaml(self, path: str) -> None:
        with open(path, 'r') as file:
            things = yaml.safe_load(file)

        nodes = {}  # store in a dict for fast graph construction

        for node in things["nodes"]:
            nodes[node["name"]] = IntersectionNode(
                name=node["name"],
                xPos=node["xpos"],
                yPos=node["ypos"],
                leftNode=node.get("left"),
                rightNode=node.get("right"),
                aheadNode=node.get("ahead"),
                uturnNode=node.get("uturn"),
            )

        self.DG.add_nodes_from([n for (_, n) in nodes.items()]) # just add the intersectionNode objects

        for node in nodes:
            u = nodes[node]
            if u.leftNode is not None:
                self.DG.add_edge(u, nodes[u.leftNode])

            if u.rightNode is not None:
                self.DG.add_edge(u, nodes[u.rightNode])

            if u.aheadNode is not None:
                self.DG.add_edge(u, nodes[u.aheadNode])

            if u.uturnNode is not None:
                self.DG.add_edge(u, nodes[u.uturnNode])

    def constructINM(self):
        # Can probably hardcode this with specific coordinates
        raise NotImplementedError

    def displayINM(self,env):
        # Display INM over environment
        raise NotImplementedError
    

def main():
    nm = NodeMap()
    nm.from_yaml('PathPlanning/IntersectionNodeMap/nodemap.yaml')
    print(nm.DG)
    print(list(nm.DG.nodes))
    print(list(nm.DG.edges))

if __name__ == '__main__':
    main()