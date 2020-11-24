import numpy as np
import pdb
# from .transition_models import get_successors

from scipy.spatial import ConvexHull


class Node:
    def __init__(self, robotState, envState, parentId):
        """ states and parent id should be np.int

        :param robotState: a 1D numpy array of length 4, representing (t, x, y, theta).
        :param envState: a Nx2 numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        """
        self.h = 0.0
        self.g = 1e10
        self.robotState = robotState
        self.envState = envState
        self.parentId = parentId

    @property
    def f(self):
        return self.h + self.g

    def isGoal(self):
        return len(self.envState) == 0

    def convexHull(self):
        """
        :return: 2D numpy array, block (x, y) locations
        """
        ch = ConvexHull(self.envState)
        return self.envState[ch.vertices]

class Graph:
    """ Assume self.vertices[0] is start
    """
    def __init__(self, holePos):
        """
        :param holePos: 2 Nx2 numpy array of [(x, y)], np.int
        """
        self.vertices = []
        # Used to checked whether a vertex exists in implicit graph
        self.verticesLUT = {}
        # Hole positions, used to calculate heuristics 
        self.holes = holePos

    def addVertex(self, robotState, envState, parentId):
        """
        :param robotState: a 1D numpy array of length 4, representing (t, x, y, theta).
        :param envState: a Nx2 numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        :return: vertex index 
        """
        stateKey = robotState.tobytes() + envState.tobytes() + np.array(parentId).tobytes()
        if stateKey in self.verticesLUT:
            return self.verticesLUT[stateKey]
        else:
            self.vertices.append(Node(robotState, envState, parentId))
            self.verticesLUT[stateKey] = len(self.vertices) - 1
            return len(self.vertices) - 1

    def getNode(self, vertexID):
        return self.vertices[vertexID]

    def getSuccessors(self, vertexID):
        """ Call functions from transition model
        """
        pass

    def diagDistance(self, vertexID):
        return max(self.vertices[vertexID])

    def computeNodeHeuristics(self, algorithm, node):
        if algorithm == '8n':
            # The maximum diagonal distance of any ball to its nearest hole
            dx = node.envState[:, 0:1] - np.transpose(self.holes[:, 0:1])
            dy = node.envState[:, 1:2] - np.transpose(self.holes[:, 1:2])
            
            # diagonal distance
            d8 = np.maximum(dx, dy)
            # to the nearest hole
            max_d8 = d8.min(axis=1)

            node.h = np.min(max_d8.max())
        else:
            raise ValueError('Distance metric not supported: {}'.format(algorithm))

    def computeHeuristics(self, algorithm):
        for n in self.vertices:
            self.computeNodeHeuristics(algorithm, n)


if __name__ == '__main__':
    g = Graph(np.array([[3, 3], [4, 4]]))
    g.addVertex(np.array([1, 1, 1, 1]), np.array([[4, 4], [8, 8], [6, 6], [4, 8]]), -1)

    # check convex hull
    ch = g.vertices[0].convexHull()
    print(ch)

    # check distance
    g.computeHeuristics('8n')
    print(g.vertices[0].h)