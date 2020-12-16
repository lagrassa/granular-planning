import heapq
import ipdb
import numpy as np

from forward.transition_models import parseAction, parseActionDTheta
from forward.node import Node, Graph


def ComputePath(OPEN, graph):
    plan = []
    plan_actions = []

    CLOSED = set()
    num_expanded = 0
    found_goal = False
    free_motion_count = 0
    total_transitions = 0
    while len(OPEN) > 0:

        print("\t1: Number of vertices:", len(OPEN))

        vertexID = heapq.heappop(OPEN)[1]
        CLOSED.add(vertexID)

        S = graph.getNode(vertexID)
        num_expanded +=1
        # Extract S with smallest f from OPEN
		
        print("VertexID is", vertexID)


        if graph.isGoal(S):    # Create function to compare two instances of class Node
            print("\nFound Goal!!!")
            found_goal = True
            while S.parentId!= -1:
                Sparent = graph.getNode(S.parentId)
                print("Robot(t-1)==Action==>Robot(t): {}==({})==>{}".format(Sparent.robotState, S.parentActionId, S.robotState))
                print("blkState(t):", S.envState.flatten())
                simAction = parseActionDTheta(S.parentActionId, graph.stepXY, graph.stepTheta)
                plan_actions.append(simAction)
                robot_state, block_states = graph.graphStateToSimState(S)
                plan.append(np.hstack([robot_state, block_states.flatten()]))
                S = Sparent
            plan_actions.reverse()
            plan.reverse()
            break

        print("\t2: Number of vertices:", len(OPEN))

        successors, fm_count, tt = graph.getSuccessors(vertexID)
        free_motion_count += fm_count
        total_transitions += tt
        # ipdb.set_trace()
        if num_expanded % 200 == 0:
            print("Expanding:{}, g={}, h={}, f={}".format(S.robotState, S.g, S.h, S.f))
        # ipdb.set_trace()
#        print("---Looking at successors to add")
        for act, s_prime in enumerate(successors):
#            print(s_prime)
            if s_prime not in CLOSED:
#                print("\t",s_prime)
                SNode = graph.getNode(s_prime)
                cost = 1.0 # TODO(???): cost from motion model?
                if SNode.g > S.g + cost:
                    SNode.g = S.g + cost
                    SNode.parentId = vertexID
                    SNode.parentActionId = act
                    # print("New path", act, S.robotState, SNode.robotState)
                    heapq.heappush(OPEN, (SNode.f, s_prime))

        print("\t3: Number of vertices:", len(OPEN),"\n")

    assert found_goal
    return plan_actions, plan, free_motion_count, total_transitions


def A_star(graph):
    OPEN = [(1e10, 0)]
    solution = []

    # goal node
    solution = ComputePath(OPEN, graph)
    return solution

