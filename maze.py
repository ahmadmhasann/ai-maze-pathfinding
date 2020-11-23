import sys
import math


class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = 0

    def __hash__(self):
        return hash(self.state)

    def __eq__(self, other):
        return self.state[0] == other.state[0] and self.state[0] == other.state[1]

    def __ne__(self, other):
        return not (self == other)


class Frontier:
    # Initially the frontier is empty
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0


# Depth First Search
class StackFrontier(Frontier):
    def remove(self, goal):
        if self.empty():
            raise Exception("empty frontier")
        else:
            # Last in first out
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node


# Breadth First Search
class QueueFrontier(Frontier):
    def remove(self, goal):
        if self.empty():
            raise Exception("empty frontier")
        else:
            # First in first out
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node


class Greedy(Frontier):
    def remove(self, goal):
        def manhatten_distance(node):
            res = tuple(map(lambda i, j: abs(i - j), node.state, goal))
            return sum(list(res))

        if self.empty():
            raise Exception("empty frontier")
        else:
            min = -1
            for fori in self.frontier:
                if min == -1:
                    min = manhatten_distance(fori)
                    node = fori
                    foriee = fori
                if manhatten_distance(fori) < min:
                    min = manhatten_distance(fori)
                    node = fori
                    foriee = fori
            self.frontier.remove(foriee)

            return node


class AStar(Frontier):
    def remove(self, goal):
        def manhatten_distance(node):
            res = tuple(map(lambda i, j: abs(i - j), node.state, goal))
            return sum(list(res))

        def g(start, current_node):
            if (
                start.state[0] == current_node.state[0]
                and start.state[1] == current_node.state[1]
            ):
                return 0

            return abs(start.state[0] - current_node.state[0]) + abs(
                start.state[1] + current_node.state[1]
            )

        for node in self.frontier:
            node.cost = manhatten_distance(node) + g(self.frontier[0], node)

        lowest_node = min(self.frontier, key=lambda x: x.cost)

        if lowest_node is None:
            raise Exception("Error calculating node that has the lowest cost function")

        self.frontier.remove(lowest_node)
        return lowest_node


class Maze:
    def __init__(self, filename):

        # Read file and set height and width of maze
        with open(filename) as f:
            contents = f.read()

        # Validate start and goal
        if contents.count("P") != 1:
            raise Exception("maze must have exactly one start point")
        if contents.count(".") != 1:
            raise Exception("maze must have exactly one goal")

        # Determine height and width of maze
        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        # Keep track of walls
        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):

                try:
                    if contents[i][j] == "P":
                        self.start = (i, j)
                        # # debuggin
                        # print(type(self.start))
                        # print(self.start)
                        row.append(False)
                    elif contents[i][j] == ".":
                        self.goal = (i, j)
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None

    def print(self):
        solution = self.solution[1] if self.solution is not None else None
        print()
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):
                if col:
                    print("%", end="")
                elif (i, j) == self.start:
                    print("P", end="")
                elif (i, j) == self.goal:
                    print(".", end="")
                elif solution is not None and (i, j) in solution:
                    print(".", end="")
                else:
                    print(" ", end="")
            print()
        print()

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1)),
        ]

        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

    def solve(self):
        """Finds a solution to maze, if one exists."""

        # Keep track of number of states explored
        self.num_explored = 0
        self.path_cost = 0

        # Initialize frontier to just the starting position
        start = Node(state=self.start, parent=None, action=None)
        frontier = AStar()
        frontier.add(start)

        # Initialize an empty explored set
        self.explored = set()

        # Keep looping until solution found
        while True:

            # If nothing left in frontier, then no path
            if frontier.empty():
                raise Exception("no solution")

            # Choose a node from the frontier
            node = frontier.remove(self.goal)
            self.num_explored += 1

            # If node is the goal, then we have a solution
            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                self.path_cost = len(actions)
                cells.reverse()
                self.solution = (actions, cells)
                return

            # Mark node as explored
            self.explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)


maze = Maze(sys.argv[1])
print("Maze:")
maze.print()
maze.solve()
print("Solution:")
maze.print()
print("States Explored:", maze.num_explored)
print("Path Cost:", maze.path_cost)
