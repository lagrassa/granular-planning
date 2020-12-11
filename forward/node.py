import numpy as np
import ipdb

from .transition_models import parseAction, transition_model
from scipy.spatial import ConvexHull
from scipy.spatial.qhull import QhullError


class Node:
    def __init__(self, robotState, envState):
        """ states and parent id should be np.int, indicating that they are quantized value

        :param robotState: a 1D integer numpy array of length 3, representing (x, y, theta).
        :param envState: a Nx2 integer numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        """
        self.h = 0.0
        self.g = 1e10
        self.robotState = robotState
        self.envState = envState
        self.parentId = -1
        self.parentActionId = -1
        self.weight = 100.0

    @property
    def f(self):
        return self.weight * self.h + self.g

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
    def __init__(self, holePos, world, stepXY, stepTheta, numActions=4, heuristicAlg='8n'):
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
        self.heuristicAlg = heuristicAlg

    def reset(self):
        self.vertices = []
        self.verticesLUT = {}

    def isGoal(self, node):
        """Reach the goal when all blocks are in the holes"""
        return self.diagDistance(node, self.holes)[1] == 0

    def addVertex(self, robotState, envState):
        """ Maintains both vertices list and verticesLUT
        :param robotState: a 1D numpy array of length 3, representing (x, y, theta).
        :param envState: a Nx2 numpy array of [(x, y) for block1, (x, y) for block 2, ...]
        :return: vertex index 
        """
        stateKey = robotState.tobytes() + envState.tobytes()
        if stateKey in self.verticesLUT:
            return self.verticesLUT[stateKey]
        else:
            self.vertices.append(Node(robotState, envState))
            self.verticesLUT[stateKey] = len(self.vertices) - 1
            self.computeNodeHeuristics(self.vertices[-1])

            return len(self.vertices) - 1

    def getNode(self, vertexID):
        return self.vertices[vertexID]

    def quantRobotState(self, robotState):
        qs = np.zeros(3)
        qs[:2] = np.round(robotState[:2] / self.stepXY)
        qs[2] = np.round(robotState[2] / self.stepTheta)
        # make heading index non negative
        qs[2] = qs[2] % ((np.pi + 1e-6) // self.stepTheta)
        return qs.astype(np.int)

    def quantBlockStates(self, blockStates):
        return np.round(blockStates / self.stepXY).astype(np.int)

    def iQuantRobotState(self, robotState):
        iqs = np.zeros(3)
        iqs[:2] = robotState[:2] * self.stepXY
        iqs[2] = robotState[2] * self.stepTheta
        return iqs

    def iQuantBlockStates(self, blockStates):
        return blockStates * self.stepXY

    def graphStateToSimState(self, n):
        """ Inverse quantization
        """
        simRobotState = self.iQuantRobotState(n.robotState)
        simBlkStates = self.iQuantBlockStates(n.envState)
        return np.concatenate((simRobotState, simBlkStates.flat))

    def simStateToGraphState(self, rState, bStates):
        return self.quantRobotState(rState), self.quantBlockStates(bStates)

    def getSuccessors(self, vertexID):
        """ Call functions from transition model
        """
        successors = []
        node = self.vertices[vertexID]
        simState = self.graphStateToSimState(node) #TODO use Steven State object
        for action in range(self.numActions):
            #simAction = parseAction(action, node.robotState[-1], self.stepXY, self.stepTheta)
            simAction = parseActionDTheta(action, self.stepXY, self.stepTheta) #uses new sim representation
            simRobotState, simBlkStates = transition_model(simState, simAction, self.world)
            # graph and sim have different representation for action
            # print("simRobotState=", simState[:3], simRobotState, action)
            # ipdb.set_trace()
            graphRobotState, graphBlkStates = self.simStateToGraphState(simRobotState, simBlkStates) #TODO convert back from Steven State
            successors.append(self.addVertex(graphRobotState, graphBlkStates))
        return successors

    def diagDistance(self, node, targets):
        # The maximum diagonal distance of any ball to its nearest hole
        dx = np.abs(node.envState[:, 0:1] - np.transpose(targets[:, 0:1]))
        dy = np.abs(node.envState[:, 1:2] - np.transpose(targets[:, 1:2]))
        
        # diagonal distance
        d8 = np.maximum(dx, dy)
        # to the nearest hole
        min_d8 = d8.min(axis=1)
        # the furthest block to the goal
        blkId = np.argmax(min_d8)
        return blkId, min_d8[blkId]

    def euclidDistance(self, node, targets):
        # The maximum Euclidean distance of any ball to its nearest hole
        dx = np.abs(node.envState[:, 0:1] - np.transpose(targets[:, 0:1]))
        dy = np.abs(node.envState[:, 1:2] - np.transpose(targets[:, 1:2]))
        
        # diagonal distance
        eucid = np.sqrt(dx ** 2 + dy ** 2)
        # to the nearest hole
        min_euclid = eucid.min(axis=1)
        # the furthest block to the goal
        blkId = np.argmax(min_euclid)
        return blkId, min_euclid[blkId]

    def computeNodeHeuristics(self, node):
        if self.heuristicAlg == '8n':
            blkId, node.h = self.diagDistance(node, self.holes)
            # distance from robot to the target block
            d = np.max(np.abs(node.envState[blkId] - node.robotState[:2]))
            d1 = 0
            if node.envState[blkId][0] > 0:
                d1 += node.robotState[0] < node.envState[blkId][0]
            else:
                d1 += node.robotState[0] > node.envState[blkId][0]
            if node.envState[blkId][1] > 0:
                d1 += node.robotState[1] < node.envState[blkId][1]
            else:
                d1 += node.robotState[1] > node.envState[blkId][1]
            node.h += d + d1
        elif self.heuristicAlg == 'sum':
            # number of blocks x epsilon admissible
            # The maximum diagonal distance of any block to its nearest hole
            dx = np.abs(node.envState[:, 0:1] - np.transpose(self.holes[:, 0:1]))
            dy = np.abs(node.envState[:, 1:2] - np.transpose(self.holes[:, 1:2]))
            
            # 1. diagonal distance heuristic
            d8 = np.maximum(dx, dy)
            # to the nearest hole
            min_d8 = d8.min(axis=1)
            # the sum of each block to the goal
            node.h = np.sum(min_d8)

            # 2. kinematic heuristic
            blkId = np.argmax(min_d8)
            node.h += np.sum(np.abs(node.envState[blkId] - node.robotState[:2]))

            d1 = 0
            if node.envState[blkId][0] > 0:
                d1 += node.robotState[0] < node.envState[blkId][0]
            elif node.envState[blkId][0] < 0:
                d1 += node.robotState[0] > node.envState[blkId][0]
            else:
                d1 += node.robotState[0] != node.envState[blkId][0]

            if node.envState[blkId][1] > 0:
                d1 += node.robotState[1] < node.envState[blkId][1]
            elif node.envState[blkId][1] > 0:
                d1 += node.robotState[1] > node.envState[blkId][1]
            else:
                d1 += node.robotState[1] != node.envState[blkId][1]

            node.h += 10 * d1
        else:
            raise ValueError('Distance metric not supported: {}'.format(self.heuristicAlg))

    def computeHeuristics(self, algorithm):
        for n in self.vertices:
            self.computeNodeHeuristics(n)


if __name__ == '__main__':
    holePos = np.array([[-2, -2],
       [-1, -2],
       [ 0, -2],
       [ 1, -2],
       [ 2, -2],
       [-2, -1],
       [-1, -1],
       [ 0, -1],
       [ 1, -1],
       [ 2, -1],
       [-2,  0],
       [-1,  0],
       [ 0,  0],
       [ 1,  0],
       [ 2,  0],
       [-2,  1],
       [-1,  1],
       [ 0,  1],
       [ 1,  1],
       [ 2,  1],
       [-2,  2],
       [-1,  2],
       [ 0,  2],
       [ 1,  2],
       [ 2,  2]], dtype=np.int)
    world = None
    stepXY = 0.1
    stepTheta = np.pi / 4

    g = Graph(holePos, world, stepXY, stepTheta)
    g.addVertex(np.array([3, 3, 1], dtype=np.int), 
                        np.array([[0, 1], [0, 2]], dtype=np.int))

    # check convex hull
    ch = g.vertices[0].convexHull()
    print(ch, ch.dtype)

    # check distance
    print(g.isGoal(g.vertices[0]))
    # print(g.vertices[0].h)
