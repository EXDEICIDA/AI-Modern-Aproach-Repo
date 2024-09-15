import heapq


class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __lt__(self, other):
        return self.path_cost < other.path_cost


def best_first_search(problem, f):

    initial_node = Node(state=problem.initial)


    frontier = [(f(initial_node), initial_node)]  # Each element is (priority, node)
    heapq.heapify(frontier)

    # Reached dictionary to track the best known path to each state
    reached = {problem.initial: initial_node}

    while frontier:
        # Pop the node with the lowest f-value (best candidate)
        _, node = heapq.heappop(frontier)

        # Goal check
        if problem.is_goal(node.state):
            return node  # Return the goal node if found

        # Expand the current node
        for child in expand(problem, node):
            s = child.state
            if s not in reached or child.path_cost < reached[s].path_cost:
                # Update the reached dictionary and add child to the frontier
                reached[s] = child
                heapq.heappush(frontier, (f(child), child))

    return None  # If no solution was found, return failure


def expand(problem, node):
    # Expanding the node by applying all possible actions
    s = node.state
    for action in problem.actions(s):
        s_prime = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s_prime)
        yield Node(state=s_prime, parent=node, action=action, path_cost=cost)


# Example of a problem class that your function would use:
class Problem:
    def __init__(self, initial, goal):
        self.initial = initial
        self.goal = goal

    def is_goal(self, state):
        return state == self.goal

    def actions(self, state):
        # Define the possible actions from the current state
        return ["action1", "action2", "action3"]

    def result(self, state, action):

        return state + action  # Placeholder example

    def action_cost(self, state, action, result_state):

        return 1  # Placeholder cost


# Example heuristic function (f)
def heuristic(node):
    return node.path_cost  # Example of a simple cost-based heuristic
