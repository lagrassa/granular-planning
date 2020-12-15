import ipdb
import numpy as np
import time
import planning.astar as astar

from envs.block_env import Simulator
from forward.node import Node, Graph
from forward.state import State
from forward.transition_models import *

start = time.time()
REPLAN = False

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


def isGoalSim(blockStates, goalRect):
    """
    :param blockStates: np.array, shape is [num_blocks, 2]
    :param goalRect: (xmin, xmax, ymin, ymax) in float, the goal area
    """
    isGoal = True
    for bs in blockStates:
        if not(goalRect[0] <= bs[0] <= goalRect[1] and 
                goalRect[2] <= bs[1] <= goalRect[3]):
            isGoal = False
            break
    return isGoal

def robotPosDiff(pos1, pos2):
    diff = pos1 - pos2
    diff[2] = pos1[2] % (2 * np.pi) - pos2[2] % (2 * np.pi)
    if diff[2] >= np.pi:
        diff[2] -= 2 * np.pi
    if diff[2] <= -np.pi:
        diff[2] += 2 * np.pi
    return diff


# Set up the simulator
# workspace top left =      [-workspace_size/2, +workspace_size/2]
#           bottom right =  [+workspace_size/2, -workspace_size/2]
#
# hole area top left =      [-goal_size/2, +goal_size/2]
#           bottom right =  [+goal_size/2, -goal_size/2]
#
workspace_size = 5
goal_size = 0.5

plan_world = Simulator(workspace_size, goal_size, gui=False, num_boxes = 2)
robot_state = [0, 0.5, 0.0]
box_states = [0.6, 0, 0, 0.6]
state = np.hstack([robot_state, box_states])
init_state = state.copy()
plan_world.set_state(state)
obs = plan_world.get_state()
assert(np.allclose(state, obs))

print("Simulator created")

# Create graph
# goal_size divided by step_xy must be an integer
step_xy = 0.1
step_theta = np.pi / 4

numGoalCells = int((goal_size + 1e-6) // step_xy)
goalCellOffset = numGoalCells // 2
goalX, goalY = np.meshgrid(np.arange(numGoalCells) - goalCellOffset, np.arange(numGoalCells) - goalCellOffset)
goal_discrete = np.stack([goalX.flat, goalY.flat], axis=1).astype(np.int)
goal = [0,0,goal_size, goal_size]
g = Graph(goal_discrete, plan_world, step_xy, step_theta,goal, cyl_radius=0.05, numActions=4, heuristicAlg='sum', collisionThresh=1e-3)

numPlans = 0
while True:
    # Not sure whether we could reuse the graph
    g.reset()

    robotState, blockStates = plan_world.get_robot_blk_states()
    simState = plan_world.get_state()
    rState = quantRobotState(robotState, step_xy, step_theta)
    bStates = quantBlockStates(blockStates, step_xy)
    print("Robot:{}, blk:{}".format(rState, bStates))
    g.addVertex(rState, bStates)
    g.getNode(0).g = 0

    # A star
    plan_actions, plan_states = astar.A_star(g)
    plan_world.close()
    world = Simulator(workspace_size, goal_size, gui=True, num_boxes = 2)
    
    if len(plan_actions) > 0:
        numPlans += 1
        print("Plan {} created: {}".format(numPlans, plan_actions))
        world.set_state(init_state)
        world.apply_action([0, 0])
        curr_state = init_state
        # ipdb.set_trace()
        for a, state in zip(plan_actions, plan_states):
            #world.set_state(curr_state)
            import ipdb; ipdb.set_trace()
            world.apply_action(a)
            print("Robot error:{}".format(np.linalg.norm(robotPosDiff(world.get_state()[:3], state[:3]))))
            print("Block error:{}".format(np.linalg.norm(world.get_state()[3:]-state[3:])))
            quantRobotSim = quantRobotState(world.get_state()[:3], step_xy, step_theta)
            quantRobotPlan = quantRobotState(state[:3], step_xy, step_theta)

            quantBlkSim = quantBlockStates(world.get_state()[3:], step_xy)
            quantBlkPlan = quantBlockStates(state[3:], step_xy)

            # print("Robot state:{},{}vs{}".format(
            #         world.get_state()[:3], 
            #         quantRobotSim, 
            #         quantRobotPlan
            #         ))

            if REPLAN and np.linalg.norm(quantRobotSim - quantRobotPlan) > 1e-6:
                print("Observe large robot pose error")
                break

            print("Block state:{},{}vs{}".format(
                    world.get_state()[3:], 
                    quantBlockStates(world.get_state()[3:], step_xy),
                    quantBlockStates(state[3:], step_xy)
                    ))

            if np.linalg.norm(quantBlkSim - quantBlkPlan) > 1e-6:
                print("Observe large block pose error")
                break

            curr_state = state
            time.sleep(0.2)
        for i in range(4):
            world.apply_action([0, 0]) #see if it falls

        if isGoalSim(world.get_state()[3:].reshape((-1, 2)), 
            [-goal_size/2, +goal_size/2, -goal_size/2, +goal_size/2]):
            print("Goal reached...")
            # ipdb.set_trace()
            break
        else:
            print("Re-planning...{}".format(world.get_state()[:3]))
            init_state = np.copy(world.get_state())
            init_state[2] %= 2 * np.pi
            world.close()
            if REPLAN:
                plan_world = Simulator(workspace_size, goal_size, gui=False, num_boxes = 2)
                plan_world.set_state(init_state)
            else:
                break
            

print(f"Total time taken: {time.time() - start:.5f}s")
