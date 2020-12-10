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
			while S.parentId!= -1:
				plan.append(S.robotState)
				S = graph.getNode(S.parentId)
			break

		successors = graph.getSuccessors(vertexID)
		print(vertexID, S.robotState, S.envState, S.g, S.h)
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

