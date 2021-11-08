import collision
import file_parse
import sampler
import visualizer
from environment_r import EnvSimulator
from tree import Tree
import rrt
import rrt_star


def test_parse_sample_visualizer():
    rc, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    points = [sampler.sample() for _ in range(10)]
    visualizer.visualize_problem(points, rc, ob, pr[0][0], pr[0][1])
    print(collision.isCollisionFree(rc, (8, 8), ob))


def test_tree():
    rc, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    points = [sampler.sample() for _ in range(10)]
    tree = Tree(rc, ob, pr[0][0], pr[0][1])
    tree.add((0, 0), (4.5, 1))
    tree.add((4.5, 1), (6, 1))
    tree.extend((4.5, 1), (4.5, 5))

    env_sim = EnvSimulator(10, 10, rc)
    env_sim.env.tree = tree
    print(env_sim.robot.robot_origin_coords)
    env_sim.env_vis.fill_robot(color="blue")
    env_sim.env.set_start_state(pr[0][0])
    env_sim.env.set_end_state(pr[0][1])
    for o in ob:
        env_sim.env.create_obstacle(o)
    env_sim.env_vis.fill_obstacles()
    for point in points:
        env_sim.env.add_point(point)
    env_sim.env_vis.fill_points()
    env_sim.robot.translate(env_sim.env.end)
    print(env_sim.robot.robot_origin_coords)
    env_sim.env_vis.fill_robot(color="green")
    env_sim.env_vis.plot_tree()
    env_sim.robot.translate((6, 7))
    print(env_sim.robot.robot_origin_coords)
    env_sim.env_vis.fill_robot()

    env_sim.env_vis.show_environment()


def test_rrt():
    rc, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    visualizer.visualize_rrt(rc, ob, pr)


def test_rrt_star():
    rc, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    visualizer.visualize_rrt_star(rc, ob, pr)


if __name__ == '__main__':
    test_rrt_star()