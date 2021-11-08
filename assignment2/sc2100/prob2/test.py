import file_parse
import visualizer
from robot import Robot
import sampler
from environment_r import EnvSimulator


def test_parse_visualizer():
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    robot = Robot(width, height)
    visualizer.visualize_problem(robot, ob, pr[0][0], pr[0][1])


def test_sampler_visualizer():
    samples = [sampler.sample() for _ in range(10)]
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    robot = Robot(width, height)
    visualizer.visualize_points(samples, robot, ob, pr[0][0], pr[0][1])


def test_collision_checker():
    samples = [sampler.sample() for _ in range(30)]
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    robot = Robot(width, height)
    visualizer.visualize_points_with_collision_check(samples, robot, ob, pr[0][0], pr[0][1])


def test_rrt():
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")

    robot = Robot(width, height)
    visualizer.visualize_rrt(robot, ob, pr[0][0], pr[0][1], 1000)


if __name__ == '__main__':
    test_rrt()
