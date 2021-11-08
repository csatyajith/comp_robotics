from environment_r import EnvSimulator
import rrt_star
import rrt


def visualize_problem(points, robot, obstacles, start, goal):
    env_sim = EnvSimulator(10, 10, robot)
    env_sim.env_vis.fill_robot(color="blue")
    env_sim.env.set_start_state(start)
    env_sim.env.set_end_state(goal)
    for o in obstacles:
        env_sim.env.create_obstacle(o)
    env_sim.env_vis.fill_obstacles()
    for point in points:
        env_sim.env.add_point(point)
    env_sim.env_vis.fill_points()
    env_sim.robot.translate(env_sim.env.end)
    env_sim.env_vis.fill_robot(color="green")
    env_sim.env_vis.show_environment()


def visualize_rrt(rc, ob, pr):
    env_sim = EnvSimulator(10, 10, rc)
    env_sim.env_vis.fill_robot(color="blue")
    env_sim.env.set_start_state(pr[0][0])
    env_sim.env.set_end_state(pr[0][1])
    for o in ob:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.translate(env_sim.env.end)
    env_sim.env_vis.fill_robot(color="green")
    env_sim.robot.translate((6, 7))
    env_sim.env_vis.fill_robot()
    success, tree, path = rrt.RRT(rc, ob, pr[0][0], pr[0][1], 100)
    print(success)
    env_sim.env.tree = tree
    env_sim.env_vis.plot_tree()
    # for key in env_sim.env.tree.tree:
    #     print(key, env_sim.env.tree.tree[key])
    env_sim.env_vis.show_environment()


def visualize_rrt_star(rc, ob, pr):
    env_sim = EnvSimulator(10, 10, rc)
    env_sim.env_vis.fill_robot(color="blue")
    env_sim.env.set_start_state(pr[0][0])
    env_sim.env.set_end_state(pr[0][1])
    for o in ob:
        env_sim.env.create_obstacle(o)

    env_sim.env_vis.fill_obstacles()
    env_sim.robot.translate(env_sim.env.end)
    env_sim.env_vis.fill_robot(color="green")
    success, tree, path = rrt_star.RRT_star(rc, ob, pr[0][0], pr[0][1], 500, 0.1, 0.4)
    print(success)
    env_sim.env.tree = tree
    env_sim.env_vis.plot_tree()
    env_sim.env_vis.show_environment()

