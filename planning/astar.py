import numpy as np

import heapq
from forward.node import Node, Graph

def ComputePath(OPEN, graph):
	path = []
	CLOSED = set()
	while len(OPEN) > 0:
		vertexID = heapq.heappush(OPEN)[1]
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
					heapq.heappush(OPEN, (SNode.h ,s_prime))
	return path

def GenerateSuccessors(S):
	s_primes = []
	# Generate all successors of Node S
	return s_primes

def A_star():
	OPEN = []
	solution = []

	# both robot state and env state have t, we
	# only need one actually because we assume they are the
	# same
	startRobState = np.array([1, 1, 1, 1], dtype=np.int)
	startEnvState = np.array([1, 10, 10, 1], dtype=np.int)

	graph = Graph()

	# add start node, always assume it is the 0th node.
	graph.addVertex(startRobState, startEnvState, -1)
	OPEN.append(0)

	# goal node
	solution = ComputePath(OPEN, graph)
	return solution

