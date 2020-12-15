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
    while len(OPEN) > 0:
        vertexID = heapq.heappop(OPEN)[1]
        CLOSED.add(vertexID)

        S = graph.getNode(vertexID)
        num_expanded +=1
        # Extract S with smallest f from OPEN
        if graph.isGoal(S):    # Create function to compare two instances of class Node
            print("Find Goal!!!")
            found_goal = True
            while S.parentId!= -1:
                Sparent = graph.getNode(S.parentId)
                print("Action:", S.parentActionId, S.robotState)
                #simAction = parseAction(S.parentActionId, Sparent.robotState[-1], graph.stepXY, graph.stepTheta)
                simAction = parseActionDTheta(S.parentActionId, graph.stepXY, graph.stepTheta)
                plan_actions.append(simAction)
                plan.append(graph.graphStateToSimState(S))
                S = Sparent
            plan_actions.reverse()
            plan.reverse()
            break

        successors = graph.getSuccessors(vertexID)
        # ipdb.set_trace()
        if num_expanded % 200 == 0:
            print("Expanding:{}, g={}, h={}, f={}".format(S.robotState, S.g, S.h, S.f))
        # ipdb.set_trace()
        for act, s_prime in enumerate(successors):
            if s_prime not in CLOSED:
                SNode = graph.getNode(s_prime)
                cost = 1.0 # TODO(???): cost from motion model?
                if SNode.g > S.g + cost:
                    SNode.g = S.g + cost
                    SNode.parentId = vertexID
                    SNode.parentActionId = act
                    # print("New path", act, S.robotState, SNode.robotState)
                    heapq.heappush(OPEN, (SNode.f, s_prime))
    assert found_goal
    return plan_actions, plan


def A_star(graph):
    OPEN = [(1e10, 0)]
    solution = []

    # goal node
    solution = ComputePath(OPEN, graph)
    return solution

