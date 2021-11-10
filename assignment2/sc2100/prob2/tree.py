import numpy as np

import collision


class Tree:
    def __init__(self, robot, obstacles, start, goal):
        self.tree = {start: set()}
        self.path_costs = {}
        self.obstacles = obstacles
        self.robot = robot
        self.goal = goal
        self.start = start
        self.eps = 0.1

    def add(self, point1, point2):
        if self.state_dist(point1, point2) > self.eps:
            self.tree[point1].add(point2)
            self.tree[point2] = set()

    def add_path_cost(self, point1, point2, cost):
        self.path_costs[(point1, point2)] = cost

    def get_cost(self, point):
        total_cost = 0
        curr = point
        while curr != self.start:
            parent_node = self.parent(curr)
            total_cost += self.path_costs[(parent_node, curr)]
            curr = parent_node
        return total_cost

    def check_rewire_valid(self, point1, point2, discretization_const=0.01):
        i = point1
        new_x, new_y, new_theta = point1
        while True:
            len_ab = self.state_dist((new_x, new_y, new_theta), point2)
            len_ratio = discretization_const / len_ab
            if len_ratio > 1:
                break
            new_x = round((1 - len_ratio) * i[0] + len_ratio * point2[0], 3)
            new_y = round((1 - len_ratio) * i[1] + len_ratio * point2[1], 3)
            new_theta = round((1 - len_ratio) * i[2] + len_ratio * point2[2], 3)
            if not collision.isCollisionFree(self.robot, (new_x, new_y, new_theta), self.obstacles):
                return False
            if self.state_dist(point1, (new_x, new_y, new_theta)) < self.state_dist(point1, point2):
                i = new_x, new_y, new_theta
            else:
                break
        return True

    def rewire(self, point, r, prior_nearest):
        best_node = prior_nearest
        node_cost = None
        min_dist = float("inf")
        for node in self.tree:
            if node == point:
                continue
            dist = self.state_dist(node, point)
            node_cost = self.get_cost(node)
            if dist < r:
                if node_cost + dist < min_dist and self.check_rewire_valid(node, point):
                    best_node = node
                    min_dist = node_cost + dist
        if best_node != prior_nearest:
            if point in self.tree[prior_nearest]:
                self.tree[prior_nearest].remove(point)
            self.tree[best_node].add(point)
            self.path_costs[(best_node, point)] = node_cost
        return best_node

    def exists(self, point):
        return True if point in self.tree else False

    def parent(self, point):
        for k, v in self.tree.items():
            if point in v:
                return k

    @staticmethod
    def wrapped_angle_diff(a1, a2):
        diff = a1 - a2
        if diff > np.pi:
            diff -= 2 * np.pi
        if diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def state_dist(self, p1, p2):
        return (((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2) + ((self.wrapped_angle_diff(p1[2], p2[2])) ** 2)) ** 0.5

    def nearest(self, point):
        min_dist = float("inf")
        min_pt = None
        for k in self.tree.keys():
            distance = self.state_dist(k, point)
            if distance < min_dist:
                min_pt = k
                min_dist = distance
        return min_pt

    def extend(self, point1, point2, discretization_const=0.01, plotter=None):
        # Discretized version
        i = point1
        new_x, new_y, new_theta = point1
        if self.state_dist(point1, point2) < self.eps:
            return i
        while True:
            len_ab = self.state_dist((new_x, new_y, new_theta), point2)
            len_ratio = discretization_const / len_ab
            if len_ratio > 1:
                break
            new_x = round((1 - len_ratio) * i[0] + len_ratio * point2[0], 3)
            new_y = round((1 - len_ratio) * i[1] + len_ratio * point2[1], 3)
            new_theta = round((1 - len_ratio) * i[2] + len_ratio * point2[2], 3)
            if not collision.isCollisionFree(self.robot, (new_x, new_y, new_theta), self.obstacles):
                break
            if plotter:
                self.robot.set_pose((new_x, new_y, new_theta))
                self.robot.transform()
                plotter.fill_any_robot(self.robot)
            if self.state_dist(point1, (new_x, new_y, new_theta)) < self.state_dist(point1, point2):
                i = new_x, new_y, new_theta
            else:
                i = point2
                break
        self.add(point1, i)
        self.add_path_cost(point1, i, self.state_dist(point1, i))
        return i
