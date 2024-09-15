# node.py
class Node:
    def __init__(self, state, parent=None, action=None, depth=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth

    def __repr__(self):
        return f"Node(state={self.state}, depth={self.depth})"
