import numpy as np
import ipdb

from .transition_models import parseAction
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError


class Node:
    def __init__(self, robotState, envState, parentId):
        """ states and parent id should be np.int, indicating that they are quantized value

        :param robotState: a 1D integer numpy array of length 4, representing (t, x, y, theta).
        :param envState: a Nx2 integer numpy array of [(x, y) for block1, (x, y) for block 2, ...]
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
        """Reach the goal when there is no blocks left in the environment"""
        return len(self.envState) == 0

    def convexHull(self):
        """ Return the convex hull of blocks' centroids if points are not colinear. Otherwise,
        return all points.
        TODO(wpu): in singular case, return the two extreme points
        :return: 2D integer numpy array, block (x, y) locations which belong to the 
        vertex of a polygon
        """
        try:
            ch = ConvexHull(self.envState)
            return self.envState[ch.vertices]
        except QhullError:
            return self.envState


class Graph:
    """ Assume self.vertices[0] is start state
    """
    def __init__(self, holePos, world, stepXY, stepTheta, numActions=4):
        """
        :param holePos: 2D Nx2 numpy array of [(x, y)], np.int
        :param world: simulator
        :param numActions: 0 forward, 1 backward, 2 rotate 45 degree clockwise, 
                            3 rotate 45 degree counter clockwise
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
        """ Maintains both vertices list and verticesLUT
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

    def quantRobotState(self, t, robotState):
        qs = np.zeros(4)
        qs[0] = t
        qs[1:3] //= self.stepXY
        qs[3] //= self.stepTheta
        return qs.astype(np.int)

    def quantBlockStates(self, blockStates):
        return (blockStates // self.stepXY).astype(np.int)

    def iQuantRobotState(self, robotState):
        iqs = np.zeros(3)
        iqs[:2] = robotState[1:3] * self.stepXY + 0.5 * self.stepXY
        iqs[2] = robotState[3] * self.stepTheta + 0.5 * self.stepTheta
        return iqs

    def iQuantBlockStates(self, blockStates):
        return blockStates * self.stepXY + 0.5 * self.stepXY

    def graphStateToSimState(self, n):
        """ Inverse quantization
        """
        simRobotState = self.iQuantRobotState(n.robotState)
        simBlkStates = self.iQuantBlockStates(n.envState)
        return np.concatenate((simRobotState, simBlkStates.flat))

    def simStateToGraphState(self, t, rState, bStates):
        return self.quantRobotState(t, rState), self.quantBlockStates(bStates)

    def getSuccessors(self, vertexID):
        """ Call functions from transition model
        """
        successors = []
        node = self.vertices[vertexID]
        simState = self.graphStateToSimState(node)
        for action in range(self.numActions):
            self.world.set_state(simState)

            # graph and sim have different representation for action
            simAction = parseAction(action, node.robotState[-1], self.stepXY, self.stepTheta)
            self.world.apply_action(simAction)
            simRobotState, simBlkStates = self.world.get_robot_blk_states()
            graphRobotState, graphBlkStates = self.simStateToGraphState(node.robotState[0] + 1, simRobotState, simBlkStates)
            successors.append(self.addVertex(graphRobotState, graphBlkStates, vertexID))
        return successors

    def computeNodeHeuristics(self, algorithm, node):
        if algorithm == '8n':
            # The maximum diagonal distance of any ball to its nearest hole
            dx = node.envState[:, 0:1] - np.transpose(self.holes[:, 0:1])
            dy = node.envState[:, 1:2] - np.transpose(self.holes[:, 1:2])
            
            # diagonal distance
            d8 = np.maximum(dx, dy)
            # to the nearest hole
            min_d8 = d8.min(axis=1)
            # the furthest block to the goal
            node.h = np.min(min_d8.max())
        else:
            raise ValueError('Distance metric not supported: {}'.format(algorithm))

    def computeHeuristics(self, algorithm):
        for n in self.vertices:
            self.computeNodeHeuristics(algorithm, n)


if __name__ == '__main__':
    holePos = np.array([[0, 0]], dtype=np.int)
    world = None
    stepXY = 0.1
    stepTheta = np.pi / 4

    g = Graph(holePos, world, stepXY, stepTheta)
    g.addVertex(np.array([1, 1, 1, 1], dtype=np.int), 
                        np.array([[4, 4], [8, 8], [6, 6], [4, 8]], dtype=np.int), -1)

    # check convex hull
    ch = g.vertices[0].convexHull()
    print(ch, ch.dtype)

    # check distance
    g.computeHeuristics('8n')
    print(g.vertices[0].h)