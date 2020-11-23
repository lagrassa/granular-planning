from ..forward import Node, Graph

def ComputePath(OPEN, s_goal):
	path = []
	CLOSED = []
	while len(OPEN) > 0:
		# Extract S with smallest f from OPEN
		if(S.isGoal()):	# Create function to compare two instances of class Node
			break
		CLOSED.append(S)
		successors = GenerateSuccessors(S)
		for s_prime in successors:
			if s_prime not in CLOSED:
#				cost =
				if s_prime.g > (S.g + cost):
					s_prime.g = S.g + cost
					OPEN.append(s_prime)
	return path

def GenerateSuccessors(S):
	s_primes = []
	# Generate all successors of Node S
	return s_primes

def A_star():
	OPEN = []
	solution = []

	graph = Graph()
	g = 0
	s_start = Node(g)
	s_goal = Node()
	OPEN.append(s_start)
	solution = ComputePath(OPEN, s_goal)
	return solution

