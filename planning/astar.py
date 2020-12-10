import heapq
import ipdb
import numpy as np

from forward.node import Node, Graph


def ComputePath(OPEN, graph):
	plan = []
	CLOSED = set()
	while len(OPEN) > 0:
		vertexID = heapq.heappop(OPEN)[1]
		CLOSED.add(vertexID)

		S = graph.getNode(vertexID)
		# Extract S with smallest f from OPEN
		if graph.isGoal(S):	# Create function to compare two instances of class Node
			print("Find Goal!!!")
			plan.append(S.robotState)
			while S.parentId!= -1:
				# TODO: We should return action, but I don't know how to get action from robotStates before
				# and after action.
				S = graph.getNode(S.parentId)
				plan.append(S.robotState)
			plan.append(S.robotState)
			break

		successors = graph.getSuccessors(vertexID)
		# ipdb.set_trace()
		print(vertexID, S.robotState, S.g, S.h)
		for s_prime in successors:
			if s_prime not in CLOSED:
				SNode = graph.getNode(s_prime)
				cost = 1.0 # TODO(???): cost from motion model?
				if SNode.g > S.g + cost:
					SNode.g = S.g + cost
					SNode.parentId = vertexID
					heapq.heappush(OPEN, (SNode.f, s_prime))
	return plan


def A_star(graph):
	OPEN = [(1e10, 0)]
	solution = []

	# goal node
	solution = ComputePath(OPEN, graph)
	return solution

