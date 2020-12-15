import heapq
import ipdb
import numpy as np

from forward.transition_models import parseAction, parseActionDTheta
from forward.node import Node, Graph


def ComputePath(OPEN, graph):
    plan = []
    CLOSED = set()
    while len(OPEN) > 0:
        vertexID = heapq.heappop(OPEN)[1]
        CLOSED.add(vertexID)

        S = graph.getNode(vertexID)
        # Extract S with smallest f from OPEN
        if graph.isGoal(S):    # Create function to compare two instances of class Node
            print("Find Goal!!!")
            while S.parentId!= -1:
                Sparent = graph.getNode(S.parentId)
                print("Action:", S.parentActionId, S.robotState)
                #simAction = parseAction(S.parentActionId, Sparent.robotState[-1], graph.stepXY, graph.stepTheta)
                simAction = parseActionDTheta(S.parentActionId, graph.stepXY, graph.stepTheta)
                plan.append(simAction)
                S = Sparent
            plan.reverse()
            break

        successors = graph.getSuccessors(vertexID)
        # ipdb.set_trace()
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
    return plan


def A_star(graph):
    OPEN = [(1e10, 0)]
    solution = []

    # goal node
    solution = ComputePath(OPEN, graph)
    return solution

