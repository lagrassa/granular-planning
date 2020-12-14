import ipdb
import numpy as np
import planning.astar as astar

from envs.block_env import Simulator
from forward.node import Node, Graph
from forward.state import Scene
from forward.transition_models import *


def quantRobotState(robotState, xyStep, thetaStep):
    """Robot from continous simulator state to discrete graph state"""
    qs = np.zeros(3)
    qs[:2] = np.round(robotState[:2] / xyStep)
    qs[2] = np.round(robotState[2] / thetaStep)
    # avoid negative heading index
    qs[2] = qs[2] % ((np.pi + 1e-6) // thetaStep)
    return qs.astype(np.int)


def quantBlockStates(blockStates, step):
    """Blocks from continous simulator state to discrete graph state"""
    return np.round(blockStates / step).astype(np.int)


# Set up the simulator
# workspace top left =      [-workspace_size/2, +workspace_size/2]
#           bottom right =  [+workspace_size/2, -workspace_size/2]
#
# hole area top left =      [-goal_size/2, +goal_size/2]
#           bottom right =  [+goal_size/2, -goal_size/2]
#
workspace_size = 5
goal_size = 0.5

world = Simulator(workspace_size, goal_size, gui=True, num_boxes = 2)
robot_state = [0, 0.5, 0.0]
box_states = [0.3, 0, 0, 0.3]
state = np.hstack([robot_state, box_states])
init_state = state.copy()
world.set_state(state)
obs = world.get_state()
assert(np.allclose(state, obs))

print("Simulator created")

# Create graph
# goal_size divided by step_xy must be an integer
step_xy = 0.1
step_theta = np.pi / 4

numGoalCells = int((goal_size + 1e-6) // step_xy)
goalCellOffset = numGoalCells // 2
goalX, goalY = np.meshgrid(np.arange(numGoalCells) - goalCellOffset, np.arange(numGoalCells) - goalCellOffset)
goal = np.stack([goalX.flat, goalY.flat], axis=1).astype(np.int)
g = Graph(goal, world, step_xy, step_theta, numActions=4, heuristicAlg='sum')

while True:
    # Not sure whether we could reuse the graph
    g.reset()

    robotState, blockStates = world.get_robot_blk_states()
    simState = world.get_state()
    rState = quantRobotState(robotState, step_xy, step_theta)
    bStates = quantBlockStates(blockStates, step_xy)
    print("Robot:{}, blk:{}".format(rState, bStates))
    g.addVertex(rState, bStates)
    g.getNode(0).g = 0

    # A star
    plan = astar.A_star(g)
    if len(plan) > 0:
        print("Plan created: {}".format(plan))
        world.set_state(init_state)
        world.apply_action([0, 0])
        ipdb.set_trace()
        for a in plan:
            world.apply_action(a)
        world.apply_action([0, 0])
        ipdb.set_trace()
        break
