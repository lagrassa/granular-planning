import numpy as np
import pdb
from .transition_models import parseAction

from scipy.spatial import ConvexHull


class Node:
    def __init__(self, robotState, envState, parentId):
        """ states and parent id should be np.int

        :param robotState: a 1D numpy array of length 4, representing (t, x, y, theta).
        :param envState: a Nx2 numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        :param parentId: index to graph's node list
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
    def __init__(self, holePos, world, stepXY, stepTheta, numActions=4):
        """
        :param holePos: 2 Nx2 numpy array of [(x, y)], np.int
        :param world: simulator
        """
        self.vertices = []
        # Used to checked whether a vertex exists in implicit graph
        self.verticesLUT = {}
        # Hole positions, used to calculate heuristics 
        self.holes = holePos
        self.world = world

        self.stepXY = stepXY
        self.stepTheta = stepTheta

        self.numActions = numActions

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

    def quantRobotState(self, robotState, t):
        qs = np.zeros(4)
        qs[1:3] //= self.stepXY
        qs[3] //= self.stepTheta
        return qs.astype(np.int)

    def quantBlockStates(self, blockStates):
        return (blockStates // self.stepXY).astype(np.int)
        
    def graphStateToSimState(self, n):
        return np.concatenate((n.robotState[1:], n.envState.flat))

    def simStateToGraphState(self, t, rState, bStates):
        return self.quantRobotState(t, rState), self.quantBlockStates(bStates)

    def getSuccessors(self, vertexID):
        """ Call functions from transition model
        """
        successors = []
        node = self.vertices[vertexID]
        simState = self.graphStateToSimState(node)
        for action in range(self.numActions):
            # import ipdb; ipdb.set_trace()
            self.world.set_state(simState)
            simAction = parseAction(action, node.robotState[-1], self.stepXY, self.stepTheta)
            self.world.apply_action(simAction)
            nextRState, nextBStates = self.world.get_robot_blk_states()
            nextRobot, nextBlks = self.simStateToGraphState(node.robotState[0] + 1, nextRState, nextBStates)
            successors.append(self.addVertex(nextRobot, nextBlks, vertexID))
        return successors

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