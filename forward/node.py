import numpy as np
# from .transition_models import get_successors

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
            # The maximum distance of any ball to its nearest hole
            dx = node.envState[:, 0:1] - np.transpose(self.holes[:, 0:1])
            dy = node.envState[:, 1:2] - np.transpose(self.holes[:, 1:2])

            min_dx = dx.min(axis=1)
            min_dy = dy.min(axis=1)

            max_d = np.maximum(min_dx, min_dy)
            node.h = np.min(max_d.max())
        else:
            raise ValueError('Distance metric not supported: {}'.format(algorithm))

    def computeHeuristics(self, algorithm):
        for n in self.vertices:
            self.computeHeuristics(n)


if __name__ == '__main__':
    g = Graph(np.array([[3, 3]]))
    g.addVertex(np.array([1, 1, 1, 1]), np.array([[]]), -1)
    print(g.vertices)