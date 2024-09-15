# ids.py
from node import Node  # Import the Node class from node.py


def iterative_deepening_search(problem):
    for depth in range(0, float('inf')):
        result = depth_limited_search(problem, depth)
        if result != "cutoff":
            return result


def depth_limited_search(problem, limit):
    frontier = [Node(problem.initial)]  # Use the imported Node class
    result = "failure"

    while frontier:
        node = frontier.pop()

        if problem.is_goal(node.state):
            return node

        if node.depth > limit:
            result = "cutoff"
        else:
            if not is_cycle(node):
                for child in expand(problem, node):
                    frontier.append(child)

    return result


def expand(problem, node):
    children = []
    for action in problem.actions(node.state):
        child_state = problem.result(node.state, action)
        child_node = Node(state=child_state, parent=node, action=action, depth=node.depth + 1)
        children.append(child_node)
    return children


def is_cycle(node):
    ancestor = node.parent
    while ancestor:
        if ancestor.state == node.state:
            return True
        ancestor = ancestor.parent
    return False


# Example of a problem class
class Problem:
    def __init__(self, initial, goal):
        self.initial = initial
        self.goal = goal

    def is_goal(self, state):
        return state == self.goal

    def actions(self, state):
        return ["action1", "action2", "action3"]

    def result(self, state, action):
        return state + action


# Example usage
if __name__ == "__main__":
    problem = Problem(initial="start_state", goal="goal_state")
    solution = iterative_deepening_search(problem)

    if solution:
        print("Goal found:", solution.state)
    else:
        print("No solution found")
