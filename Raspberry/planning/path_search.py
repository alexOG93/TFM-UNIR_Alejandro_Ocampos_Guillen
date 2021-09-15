import numpy as np
import sys
import os
import heapq


class RobotState:
    def __init__(
            self, position, description: str = ""
    ):
        self.position = position
        self.description = description

    def get_successors(
            self, map_matrix: np.array):

        def is_valid(position, map_matrix):
            valid = map_matrix[position[0]][position[1]]
            return valid

        assert self.position is not None

        states = []

        # Left
        if self.position[0] > 0:
            position = (self.position[0] - 1, self.position[1])

            if is_valid(position, map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Right
        if self.position[0] < map_matrix.shape[0] - 1:
            position = (self.position[0] + 1, self.position[1])

            if is_valid(position, map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Up
        if self.position[1] > 0:
            position = (self.position[0], self.position[1] - 1)

            if is_valid(position, map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Down
        if self.position[1] < map_matrix.shape[1] - 1:
            position = (self.position[0], self.position[1] + 1)

            if is_valid(position, map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))
        # Down left
        if self.position[0] > 0 and self.position[1] > 0:
            position = (self.position[0] - 1, self.position[1] - 1)

            if is_valid(position, map_matrix) \
                    and is_valid((self.position[0] - 1, self.position[1]), map_matrix) \
                    and is_valid((self.position[0], self.position[1] - 1), map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Up right
        if self.position[0] < map_matrix.shape[0] - 1 and self.position[1] > 0:
            position = (self.position[0] + 1, self.position[1] - 1)

            if is_valid(position, map_matrix) \
                    and is_valid((self.position[0] + 1, self.position[1]), map_matrix) \
                    and is_valid((self.position[0], self.position[1] - 1), map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Down right
        if self.position[0] < map_matrix.shape[0] - 1 and self.position[1] < map_matrix.shape[1] - 1:
            position = (self.position[0] + 1, self.position[1] + 1)

            if is_valid(position, map_matrix) \
                    and is_valid((self.position[0] + 1, self.position[1]), map_matrix) \
                    and is_valid((self.position[0], self.position[1] + 1), map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))

        # Down left
        if self.position[0] > 0 and self.position[1] < map_matrix.shape[1] - 1:
            position = (self.position[0] - 1, self.position[1] + 1)

            if is_valid(position, map_matrix) \
                    and is_valid((self.position[0] - 1, self.position[1]), map_matrix) \
                    and is_valid((self.position[0], self.position[1] + 1), map_matrix):
                states.append(RobotState(
                    position,
                    "move robot to row %d and column %d" % (
                        position[1], position[0]
                    )
                ))
                
        return states

    def heuristic_value(self, goal_state):

        def manhattan_dist(a, b):
            x_dist = abs(b[0] - a[0])
            y_dist = abs(b[1] - a[1])
            return x_dist + y_dist

        def euclidean_dist(a, b):
            x_dist = b[0] - a[0]
            y_dist = b[1] - a[1]
            return np.sqrt(x_dist*x_dist + y_dist*y_dist)

        def diagonal_dist(a, b):
            x_dist = abs(b[0] - a[0])
            y_dist = abs(b[1] - a[1])
            return (x_dist + y_dist) + (np.sqrt(2) - 2)*np.amin([x_dist, y_dist])

        if self == goal_state:
            return 0

        h = euclidean_dist(self.position, goal_state.position)

        return h

    def get_cost(self, parent):
        column_step = self.position[0] - parent.position[0]
        row_step = self.position[1] - parent.position[1]
        # cost = abs(column_step) + abs(row_step)
        cost = np.sqrt(column_step*column_step + row_step*row_step)
        return cost

    def is_goal_state(self, goal_state):
        return np.all(self.position == goal_state.position)

    def __hash__(self):
        return hash(str(self.__dict__))

    def __eq__(self, other: object):
        if not isinstance(other, RobotState):
            return False

        return self.position == other.position

    def __lt__(self, other: object):
        return False

    def __str__(self):
        return self.description


def a_star_search(
    map_matrix: np.array,
    init_state: RobotState, goal_state: RobotState):

    heap = [(init_state.heuristic_value(goal_state), None, init_state)]

    opened = {}
    closed = {}

    opened[init_state] = (0, None)

    steps = 0

    while len(heap) > 0:
        _, parent, current = heapq.heappop(heap)

        if current not in opened:
            continue
        steps += 1
        cost, _ = opened[current]
#         print(f"Current opened is: {current}")
#         print(f"Its cost is: {cost}\n")
        del opened[current]

        if current.is_goal_state(goal_state):
            closed[current] = (cost, parent)
            closed[goal_state] = (cost, current)
            break

        successors = current.get_successors(map_matrix)

        for successor in successors:
            successor_cost = cost + successor.get_cost(current)
#             print(f"Successor opened is: {successor}")
#             print(f"Its cost is: {successor_cost}")
            assert (successor not in opened or successor not in closed)

            if (successor in opened and opened[successor][0] <= successor_cost) \
              or (successor in closed and closed[successor][0] <= successor_cost):
                continue

            if successor in closed:
                del closed[successor]

            opened[successor] = (successor_cost, parent)

            heur_cost = successor_cost
            heur_cost += successor.heuristic_value(goal_state)
#             print(f"Its heuristic cost is: {heur_cost}")

            heapq.heappush(heap, (heur_cost, current, successor))

        closed[current] = (cost, parent)
#         print("All successors checked out \n")
    if goal_state not in closed:
        return sys.maxsize, [], -1

    path = []
    state = goal_state

    while state != None:
        _, parent = closed[state]
        path.append(state)
        state = parent

    path.reverse()

    return closed[goal_state][0], path, steps


def find_path(binary_map, init_pos, goal_pos, verbose=False, verbose_file=""):
    init_state = RobotState(
        (init_pos[0], init_pos[1]), description="robot initial state in row %d and column %d" % (init_pos[1], init_pos[0])
    )

    goal_state = RobotState(
        (goal_pos[0], goal_pos[1]), description="robot target state in row %d and column %d" % (goal_pos[1], goal_pos[0])
    )

    cost, path, steps = a_star_search(binary_map, init_state, goal_state)

    if len(path) > 0:
        if verbose:
            if verbose_file:
                original_stdout = sys.stdout
                
                directory = os.path.dirname(__file__)
                directory = os.path.join(directory, "output")
                verbose_file = os.path.join(directory, verbose_file)
                if not os.path.exists("output"):
                    os.makedirs("output")

                fout = open(verbose_file, 'w+')
                sys.stdout = fout

            print("\nThe following plan was found:\n")

            for i, state in enumerate(path):
                print("\t%02d - %s" % (i, str(state)))

            print("\nTotal cost of plan: %.2f" % cost)
            print("Number of steps to find the plan: %d\n" % steps)

            if verbose_file:
                sys.stdout = original_stdout
                fout.close()

    else:
        raise ValueError("No plan was found for given configuration")

    return cost, path, steps
