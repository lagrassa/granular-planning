import numpy as np

import heapq
from forward.node import Node, Graph

def ComputePath(OPEN, graph):
	path = []
	CLOSED = set()
	while len(OPEN) > 0:
		vertexID = heapq.heappop(OPEN)[1]
		CLOSED.add(vertexID)

		S = graph.getNode(vertexID)
		# Extract S with smallest f from OPEN
		if(S.isGoal()):	# Create function to compare two instances of class Node
			break

		successors = graph.getSuccessors(vertexID)
		for s_prime in successors:
			if s_prime not in CLOSED:
				SNode = graph.getNode(s_prime)
				cost = 1.0 # TODO: cost from motion model?
				if SNode.g > S.g + cost:
					SNode.g = S.g + cost
					heapq.heappush(OPEN, (SNode.h, s_prime))
	return path


def A_star(graph):
	OPEN = [(1e10, 0)]
	solution = []

	# goal node
	solution = ComputePath(OPEN, graph)
	return solution

