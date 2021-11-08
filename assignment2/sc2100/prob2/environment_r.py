from typing import List

from matplotlib import pyplot as plt

from robot import Robot


class Environment:

    def __init__(self, n_rows, n_cols, tree=None):
        self.n_rows = n_rows
        self.n_cols = n_cols
        self.start = None
        self.end = None
        self.obstacles = []
        self.points = []
        self.tree = tree

    def set_start_state(self, coords):
        x, y, theta = coords
        self.start = (x, y, theta)

    def set_end_state(self, coords):
        x, y, theta = coords
        self.end = (x, y, theta)

    def create_obstacle(self, obstacle_coords=List[tuple]):
        self.obstacles.append(obstacle_coords)

    def add_point(self, coords):
        self.points.append((coords[0], coords[1]))


class EnvSimulator:
    def __init__(self, env_x, env_y, robot):
        self.env = Environment(env_x, env_y)
        self.robot = robot
        self.env_vis = EnvironmentVisualizer(self.env, self.robot)


class EnvironmentVisualizer:
    def __init__(self, environment: Environment, robot: Robot):
        self.environment = environment
        self.ax = None
        self.configure_plot()
        self.robot = robot
        self.box_environment()

    @staticmethod
    def show_environment():
        plt.show()

    def configure_plot(self):
        fig = plt.figure(figsize=(7, 7))
        self.ax = plt.axes()
        self.ax.set_aspect("equal")
        return fig

    def box_environment(self):
        for row in range(self.environment.n_rows):
            for cell in range(self.environment.n_cols):
                i = row
                j = cell
                if i == 0:
                    self.ax.plot([0, (j + 1)], [i, i], color="k")
                if j == self.environment.n_cols - 1:
                    self.ax.plot([(j + 1), (j + 1)], [i, (i + 1)], color="k")
                if i == self.environment.n_rows - 1:
                    self.ax.plot([(j + 1), j], [(i + 1), (i + 1)], color="k")
                if j == 0:
                    self.ax.plot([j, j], [(i + 1), i], color="k")

    def fill_points(self):
        for point in self.environment.points:
            self.ax.plot(point[0], point[1], "co")

    def fill_environment(self):
        # self.fill_start_and_goal_coords()
        self.fill_obstacles()
        self.fill_robot()
        self.fill_points()

    def fill_obstacles(self):
        for obstacle in self.environment.obstacles:
            self.ax.fill([a[0] for a in obstacle], [a[1] for a in obstacle], color="k")

    def fill_robot(self, color="blue"):
        self.ax.fill([a[0] for a in self.robot.robot_current_state], [a[1] for a in self.robot.robot_current_state],
                     color=color)

    def plot_path(self, path):
        for i in range(len(path) - 1):
            self.ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color="k")

    def plot_tree(self):
        for k, v in self.environment.tree.tree.items():
            for pt in v:
                self.ax.plot([k[0], pt[0]], [k[1], pt[1]], color="k")
