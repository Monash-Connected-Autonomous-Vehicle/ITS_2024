import sys
import pathlib
import matplotlib.pyplot as plt
import numpy as np
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from MRNEnv.environment import RasterEnv

class IntersectionNode:

    def __init__(self, xPos, yPos, leftNode, rightNode,
                 forwardNode, uTurnNode,nodeArray):
        self.xPos = xPos
        self.yPos = yPos
        self.leftNode = leftNode
        self.rightNode = rightNode
        self.forwardNode = forwardNode
        self.uTurnNode = uTurnNode
        self.nodeArray = nodeArray

    def constructINM(self):
        # Can probably hardcode this with specific coordinates
        print("Hello World!")

    def displayINM(self,env):
        # Display INM over environment
        print("Hello World!")


def main():
    print("Hello World!")

if __name__ == '__main__':
    main()