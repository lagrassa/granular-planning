import numpy as np

from ..forward import Node, Graph

def ComputePath(OPEN, graph):
	path = []
	CLOSED = set()
	while len(OPEN) > 0:
		vertexID = OPEN.pop()
		CLOSED.add(vertexID)

		S = graph.getNode(vertexID)
		# Extract S with smallest f from OPEN
		if(S.isGoal()):	# Create function to compare two instances of class Node
			break

		successors = graph.getSuccessors(vertexID)
		for s_prime in successors:
			if s_prime not in CLOSED:
				SNode = self.getNode(s_prime)
				cost = ?
				if SNode.g > S.g + cost:
					SNode.g = S.g + cost
					OPEN.append(s_prime)
	return path

def GenerateSuccessors(S):
	s_primes = []
	# Generate all successors of Node S
	return s_primes

def A_star():
	# TODO: change OPEN to heapq
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

