from environment_r import EnvSimulator
import collision


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
