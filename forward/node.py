import numpy as np
import ipdb

from .transition_models import parseAction, transition_model, parseActionDTheta
from .state import State
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
    def __init__(self, holePos, world, stepXY, stepTheta, goal, cyl_radius = 0.05, numActions=4, heuristicAlg='8n', collisionThresh=1e-3):
        """
        :param DEPRECATED for all but heuristics: holePos: 2D Nx2 numpy array of [(x, y)], np.int
        :param holePos: array: [x,y,w,h] (x,y) pos and (w, h)
        :param world: simulator
        :param numActions: 0 forward, 1 backward, 2 rotate 45 degree clockwise, 
                            3 rotate 45 degree counter clockwise
        :param heuristicAlg: string of the heuristic to use ('8n' or 'sum')
        :param collisionThresh: threshold to use for collision checking in meters
        """
        self.vertices = []
        # Used to checked whether a vertex exists in implicit graph
        self.verticesLUT = {}
        # Hole positions, used to calculate heuristics 
        self.holes = holePos
        self.cyl_radius = cyl_radius

        self.hole_center = goal[0:2]
        self.w  = goal[2]
        self.h = goal[3]
        assert (self.w == self.h) #square goal atm
        self.world = world

        self.stepXY = stepXY
        self.stepTheta = stepTheta

        self.numActions = numActions
        self.heuristicAlg = heuristicAlg

        self.collisionThresh = collisionThresh

    def reset(self):
        self.vertices = []
        self.verticesLUT = {}

    def viz_node(self, node):
        old_state = self.world.get_state()
        simState = self.graphStateToSimState(node)
        self.world.set_state(simState)
        input("Visualize world, OK?")

    def isGoal(self, node):
        """Reach the goal when all blocks are in the holes"""
        simState = self.graphStateToSimState(node)
        coords = simState[1]
        is_goal_result = True
        for dim, goal_dim in zip([0,1], [self.w, self.h]):
            if np.any((self.hole_center[dim]-goal_dim/2)>coords[:,dim]+self.cyl_radius):
                is_goal_result = False
            if np.any((self.hole_center[dim]+goal_dim/2)<coords[:,dim]-self.cyl_radius):
                is_goal_result = False


        old_is_goal_result =  self.diagDistance(node, self.holes)[1] == 0
        if (old_is_goal_result != is_goal_result):
            print("Different results between old and new result")
            import ipdb; ipdb.set_trace()
        return is_goal_result

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
        """
        Inverse quantization
        """
        return self.iQuantRobotState(n.robotState), self.iQuantBlockStates(n.envState)

    def simStateToGraphState(self, rState, bStates):
        """
        Quantize robot state to graph state format
        """
        return self.quantRobotState(rState), self.quantBlockStates(bStates)

    def getSuccessors(self, vertexID):
        """ 
        Get succesors from transition model
        """
        successors = []
        # Get the parent state representation
        node = self.vertices[vertexID]
        parentRobotState, parentBlockStates = self.graphStateToSimState(node)
        simState = State(self.world) 
        free_motion_count = 0
        total_transitions = 0
        # iterate through all actions and apply to parent state
        for action_type in range(self.numActions):
            simAction = parseActionDTheta(action_type, self.stepXY, self.stepTheta)
            # apply action
            simRobotState, simBlkStates, count = transition_model(simState,
                                                           parentRobotState,
                                                           parentBlockStates,
                                                           simAction,
                                                           threshold=self.collisionThresh,
                                                           sim_flag=False)
            free_motion_count += count
            total_transitions += 1
            # convert back to graph state format
            graphRobotState, graphBlkStates = self.simStateToGraphState(simRobotState, simBlkStates)
            successors.append(self.addVertex(graphRobotState, graphBlkStates))
        return successors, free_motion_count, total_transitions

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
            if node.h > 0:
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

                node.h += d1
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
