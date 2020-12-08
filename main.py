import ipdb
import numpy as np
import planning.astar as astar

from envs.block_env import Simulator
from forward.node import Node, Graph
from forward.state import EnvRepresentation 
from forward.transition_models import *


def quantRobotState(robotState, xy_step, theta_step):
    qs = np.zeros(3)
    qs[:2] = robotState[:2] // xy_step
    qs[2] = robotState[2] // theta_step
    qs[2] = qs[2] % (np.pi / theta_step)
    return qs.astype(np.int)

def quantBlockStates(blockStates, step):
    return (blockStates // step).astype(np.int)

# Set up the simulator
# workspace top left = [-workspace_size/2, +workspace_size/2]
# bottom right = [-workspace_size/2, -workspace_size/2]
workspace_size = 5
goal_size = 0.3

world = Simulator(workspace_size, goal_size, gui=True, num_boxes = 2)
robot_state = [-1.0, 0.0, 0.0]
box_states = [1, 0, 0, 1]
state = np.hstack([robot_state, box_states])
world.set_state(state)
obs = world.get_state()
assert(np.allclose(state, obs))

print("Simulator created")

# Create graph
# goal_size divided by step_xy must be an integer
step_xy = 0.1
step_theta = np.pi / 4

numGoalCells = int(goal_size // step_xy)
goalX, goalY = np.meshgrid(np.arange(numGoalCells), np.arange(numGoalCells))
goal = np.stack([goalX.flat, goalY.flat], axis=1).astype(np.int)
g = Graph(goal, world, step_xy, step_theta)

while True:
    g.reset()

    robotState, blockStates = world.get_robot_blk_states()
    rState = quantRobotState(robotState, step_xy, step_theta)
    bStates = quantBlockStates(blockStates, step_xy)
    print("Robot:{}, blk:{}".format(rState, bStates))
    g.addVertex(rState, bStates)
    g.getNode(0).g = 0

    # A star
    plan = astar.A_star(g)
    print("Plan created: {}".format(plan))

