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
    def __init__(self):
        self.vertices = []
        # Used to checked whether a vertex exists in implicit graph
        self.verticesLUT = {}

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

    def getSuccessor(self, vertexID):
        pass

if __name__ == '__main__':
    g = Graph()
    g.addVertex(np.array([1, 1, 1, 1]), np.array([[]]), -1)
    print(g.vertices)