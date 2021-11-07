import sampler
from tree import Tree


def RRT(robot, obstacles, start, goal, n_iter):
    rrt_tree = Tree(robot, obstacles, start, goal)
    radius = 0.3
    success = False
    for _ in range(n_iter):
        random_point = sampler.sample()
        nearest_in_tree = rrt_tree.nearest(random_point)
        extended_point = rrt_tree.extend(nearest_in_tree, random_point)
        print("Sampled point: ", random_point, "Nearest in tree: ", nearest_in_tree, "Extended until:", extended_point)
        print("Updated tree:", rrt_tree.tree, "\n")
        dist = Tree.euclidean_dist(extended_point, goal)
        if dist < 2 * radius:
            rrt_tree.add(extended_point, goal)
            # print('success')
            success = True
            break

    path = None
    if success:
        path = [goal]
        while path[-1] != start:
            path.append(rrt_tree.parent(path[-1]))
        path = path[::-1]
    return success, rrt_tree, path
