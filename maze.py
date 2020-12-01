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
            for node in self.frontier:
                if min == -1:
                    min = manhatten_distance(fori)
                    min_node = node
                if manhatten_distance(fori) < min:
                    min = manhatten_distance(fori)
                    min_node = node
            self.frontier.remove(min_node)
            return min_node


class AStar(Frontier):
    def remove(self, goals, current_goal, current_explored):
        def manhatten_distance(node, goal):
            res = tuple(map(lambda i, j: abs(i - j), node.state, goal))
            return sum(list(res))

        def g(start, current_node):
            if (
                start.state[0] == current_node.state[0]
                and start.state[1] == current_node.state[1]
            ):
                return 0

            return abs(start.state[0] - current_node.state[0]) + abs(
                start.state[1] - current_node.state[1]
            )
        min_cost = -1
        lowest_node = self.frontier[0]
        if current_goal is not None:
            for node in self.frontier:
                if min_cost != -1 and manhatten_distance(node, current_goal) + g(self.frontier[0], node) < min_cost:
                    min_cost = manhatten_distance(node, current_goal) + g(self.frontier[0], node)
                    lowest_node = node
                elif min_cost == -1:
                    min_cost = manhatten_distance(node, current_goal) + g(self.frontier[0], node)
                    lowest_node = node
        else:
            for goal in goals:
                for node in self.frontier:
                    if min_cost != -1 and manhatten_distance(node, goal) + g(self.frontier[0], node) < min_cost:
                        current_goal = goal
                        min_cost = manhatten_distance(node, goal) + g(self.frontier[0], node)
                        lowest_node = node
                    elif min_cost == -1:
                        min_cost = manhatten_distance(node, goal) + g(self.frontier[0], node)
                        lowest_node = node

        if lowest_node is None:
            raise Exception("Error calculating node that has the lowest cost function")

        self.frontier.remove(lowest_node)
        if (current_goal == lowest_node.state):
            goals.remove(current_goal)
            current_explored = set()
            print (current_goal)
            current_goal = None
        
        return lowest_node, goals, current_goal, current_explored


class Maze:
    def __init__(self, filename):

        # Read file and set height and width of maze
        with open(filename) as f:
            contents = f.read()

        # Validate start and goal
        if contents.count("P") != 1:
            raise Exception("maze must have exactly one start point")

        # Determine height and width of maze
        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        # Keep track of walls
        self.walls = []
        self.goals = []
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
                        self.goals.append ((i, j))
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
                elif (i, j) in self.goals:
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
        self.current_explored = set()
        current_goal = None

        # Keep looping until solution found
        while True:

            # If nothing left in frontier, then no path
            if frontier.empty():
                for goal in self.goals:
                    print (goal)
                raise Exception("no solution")

            # Choose a node from the frontier
            node, self.goals, current_goal, self.current_explored = frontier.remove(self.goals, current_goal, self.current_explored)
            self.num_explored += 1
            print(len(self.goals))

            # If node is the goal, then we have a solution
            if len(self.goals) == 1:
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
            self.current_explored.add(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in self.current_explored:
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
