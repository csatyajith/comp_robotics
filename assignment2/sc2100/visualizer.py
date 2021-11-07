from assignment2.sc2100.environment import EnvSimulator


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


