import sampler
from tree import Tree


def RRT_star(robot, obstacles, start, goal, n_iter, radius_around_goal=0.4, radius_rewire=0.5):
    rrt_star_tree = Tree(robot, obstacles, start, goal)
    rg = radius_around_goal
    success = False
    rewires = 0
    for _ in range(n_iter):
        random_point = sampler.sample()
        nearest_in_tree = rrt_star_tree.nearest(random_point)
        extended_point = rrt_star_tree.extend(nearest_in_tree, random_point)
        best_extension = rrt_star_tree.rewire(extended_point, radius_rewire, nearest_in_tree)
        if best_extension != extended_point:
            rewires += 1
            # print("Rewiring {} successful. Original: {}   Rewired: {}".format(rewires, extended_point, best_extension))
        dist = rrt_star_tree.state_dist(best_extension, goal)
        if dist < 2 * rg:
            rrt_star_tree.add(best_extension, goal)
            rrt_star_tree.add_path_cost(best_extension, goal, dist)
            # print('success')
            success = True
            break

    path = None
    if success:
        path = [goal]
        while path[-1] != start:
            path.append(rrt_star_tree.parent(path[-1]))
        path = path[::-1]
    return success, rrt_star_tree, path
