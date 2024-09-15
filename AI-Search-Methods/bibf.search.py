import heapq
from node import Node  # Assuming you have a Node class in node.py


def bibf_search(problemF, fF, problemB, fB):
    nodeF = Node(problemF.initial)  # Node for the start state
    nodeB = Node(problemB.initial)  # Node for the goal state

    # Priority queues for forward and backward search
    frontierF = [(fF(nodeF), nodeF)]
    frontierB = [(fB(nodeB), nodeB)]
    heapq.heapify(frontierF)
    heapq.heapify(frontierB)

    # Lookup tables for reached states
    reachedF = {nodeF.state: nodeF}
    reachedB = {nodeB.state: nodeB}

    solution = None

    while not terminated(solution, frontierF, frontierB):
        # Determine whether to proceed in the forward or backward direction
        if fF(frontierF[0][1]) < fB(frontierB[0][1]):
            solution = proceed('F', problemF, frontierF, reachedF, reachedB, solution)
        else:
            solution = proceed('B', problemB, frontierB, reachedB, reachedF, solution)

    return solution


def proceed(dir, problem, frontier, reached, reached2, solution):
    # Expand the node on the frontier
    _, node = heapq.heappop(frontier)

    for child in expand(problem, node):
        s = child.state

        # If this is a new state or a better path, update reached
        if s not in reached or child.path_cost < reached[s].path_cost:
            reached[s] = child
            heapq.heappush(frontier, (child.path_cost, child))

            # If this state is also in the other search's reached set, join the paths
            if s in reached2:
                solution2 = join_nodes(dir, child, reached2[s])
                if solution is None or solution2.path_cost < solution.path_cost:
                    solution = solution2

    return solution


# Helper functions
def terminated(solution, frontierF, frontierB):
    # Determine whether to terminate the search (based on custom logic)
    return solution is not None or not frontierF or not frontierB


def join_nodes(dir, node1, node2):
    # Combine two nodes from forward and backward search into a solution
    if dir == 'F':
        # Forward direction: join paths from node1 to node2
        return node1.path_to_root() + node2.path_to_root()[::-1]
    else:
        # Backward direction: join paths from node2 to node1
        return node2.path_to_root() + node1.path_to_root()[::-1]


def expand(problem, node):
    # This function generates child nodes
    children = []
    for action in problem.actions(node.state):
        child_state = problem.result(node.state, action)
        child_node = Node(state=child_state, parent=node, action=action,
                          path_cost=node.path_cost + problem.action_cost(node.state, action))
        children.append(child_node)
    return children


# Example usage
class Problem:
    def __init__(self, initial, goal):
        self.initial = initial
        self.goal = goal

    def is_goal(self, state):
        return state == self.goal

    def actions(self, state):
        return ["action1", "action2"]

    def result(self, state, action):
        return state + action

    def action_cost(self, state, action):
        return 1  # Example cost


# Example heuristic functions for forward and backward searches
def heuristicF(node):
    return node.path_cost  # Forward heuristic


def heuristicB(node):
    return node.path_cost  # Backward heuristic


# Example of running the BIBF search
if __name__ == "__main__":
    problemF = Problem(initial="start", goal="goal")
    problemB = Problem(initial="goal", goal="start")

    solution = bibf_search(problemF, heuristicF, problemB, heuristicB)
    if solution:
        print("Solution found:", solution)
    else:
        print("No solution found")
