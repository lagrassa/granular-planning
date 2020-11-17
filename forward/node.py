# from .transition_models import get_successors

class Node:
    def __init__(self, robotState=None, envState=None):
        """
        :param robotState: if provided, should be a 1D numpy array of length 4, representing (t, x, y, theta).
        :param envState: if provided, should be a Nx2 numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        """
        self.h = 0.0
        self.g = 1e10
        self.robotState = robotState
        self.envState = envState
        self.parent_id = None

    @property
    def f(self):
        return self.h + self.g

    def isGoal(self):
        return len(self.envState) == 0


class Graph:
    def __init__(self):
        self.vertices = {}

    def getSuccessor(self, vertex_id):
        pass

if __name__ == '__main__':
    n = Node()
    print(n.f)