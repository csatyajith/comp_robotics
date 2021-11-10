import collision
import rrt
import rrt_star
import utils
from environment_r import EnvSimulator


def visualize_problem(robot, obstacles, start, goal):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for o in obstacles:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")
    env_sim.env_vis.show_environment()


def visualize_points_with_collision_check(points, robot, obstacles, start, goal):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for point in points:
        env_sim.robot.set_pose(point)
        env_sim.robot.transform()
        if collision.isCollisionFree(robot, point, obstacles):
            env_sim.env_vis.fill_robot(color="violet")
        else:
            env_sim.env_vis.fill_robot(color="red")
    for o in obstacles:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")
    env_sim.env_vis.show_environment()


def visualize_points(points, robot, obstacles, start, goal):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for point in points:
        env_sim.robot.set_pose(point)
        env_sim.robot.transform()
        env_sim.env_vis.fill_robot(color="violet")

    for o in obstacles:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")
    env_sim.env_vis.show_environment()


def visualize_rrt(robot, obstacles, start, goal, n_iterations):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for o in obstacles:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")

    success, tree, path = rrt.RRT(robot, obstacles, start, goal, n_iterations, plotter=None)
    print(success)
    env_sim.env.tree = tree
    env_sim.env_vis.animate_tree_construction(tree.tree_list)
    # for key in env_sim.env.tree.tree:
    #     print(key, env_sim.env.tree.tree[key])
    env_sim.env_vis.show_environment()
    return path


def visualize_rrt_star(robot, obstacles, start, goal, n_iterations):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for o in obstacles:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")

    success, tree, path = rrt_star.RRT_star(robot, obstacles, start, goal, n_iterations)
    print(success)
    env_sim.env.tree = tree
    env_sim.env_vis.plot_tree()
    # for key in env_sim.env.tree.tree:
    #     print(key, env_sim.env.tree.tree[key])
    env_sim.env_vis.show_environment()
    return path


def visualize_path(robot, ob, pr, path):
    env_sim = EnvSimulator(10, 10, robot)

    env_sim.env.set_start_state(pr[0][0])
    env_sim.env.set_end_state(pr[0][1])

    env_sim.robot.set_pose(env_sim.env.start)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="blue")

    for o in ob:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()

    env_sim.robot.set_pose(env_sim.env.end)
    env_sim.robot.transform()
    env_sim.env_vis.fill_robot(color="green")

    env_sim.env_vis.animate_path(path, robot)
    # print(len(path))
    # for i in range(len(path) - 1):
    #     env_sim.env_vis.ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]])
    #     env_sim.robot.set_pose(path[i+1])
    #     env_sim.robot.transform()
    #     env_sim.env_vis.fill_robot(color="violet")

    env_sim.env_vis.show_environment()


def discretize_points_for_animation(path):
    discretization_const = 0.05
    new_path = []
    for i in range(len(path) - 1):
        point1 = path[i]
        point2 = path[i + 1]
        i = point1
        new_x, new_y, new_theta = point1
        while True:
            len_ab = utils.state_dist((new_x, new_y, new_theta), point2)
            len_ratio = discretization_const / len_ab
            new_x = round((1 - len_ratio) * i[0] + len_ratio * point2[0], 3)
            new_y = round((1 - len_ratio) * i[1] + len_ratio * point2[1], 3)
            new_theta = round((1 - len_ratio) * i[2] + len_ratio * point2[2], 3)
            if utils.state_dist(point1, (new_x, new_y, new_theta)) < utils.state_dist(point1, point2):
                i = new_x, new_y, new_theta
                new_path.append(i)
            else:
                new_path.append(point2)
                break
    return new_path
