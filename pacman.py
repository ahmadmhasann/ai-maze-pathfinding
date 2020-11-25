from maze import Maze
import sys


class Pacman(Maze):
    def __init__(self, filename):
        self.goals = []

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
                        self.goals.append((i, j))
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None

    def solve(self):
        for goal in self.goals:
            super().solve(self)

    def print(self):
        super().print()


if __name__ == "__main__":
    pacman = Pacman(sys.argv[1])
    print("Maze:")
    pacman.print()
    pacman.solve()
    print("Solution:")
    pacman.print()
    print("States Explored:", pacman.num_explored)
    print("Path Cost:", pacman.path_cost)