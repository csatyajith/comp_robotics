import visualizer_utils
import file_parse
import visualizer
from robot import Robot
import sampler
import rrt
import rrt_star


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
    path = visualizer.visualize_rrt(robot, ob, pr[0][0], pr[0][1], 1000)
    if path is not None:
        disc_path = visualizer_utils.discretize_points_for_animation(path)
        # animation.animate_plot(disc_path)
        visualizer.visualize_path(robot, ob, pr, disc_path, file_name="rrt.gif")


def test_rrt_star():
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    robot = Robot(width, height)
    path = visualizer.visualize_rrt_star(robot, ob, pr[0][0], pr[0][1], 1000)

    if path is not None:
        disc_path = visualizer_utils.discretize_points_for_animation(path)
        visualizer.visualize_path(robot, ob, pr, disc_path, file_name="rrt_star.gif")


def test_rrt_success_rate():
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    successes = []
    iterations = []

    robot = Robot(width, height)
    for i in range(10, 201, 10):
        print("Measuring with {} iterations".format(i))
        s = []
        for j in range(20):
            success, tree, path = rrt.RRT(robot, ob, pr[0][0], pr[0][1], i, radius_around_goal=0.2)
            s.append(1 if success else 0)
        iterations.append(i)
        successes.append(sum(s)/len(s))
    print("Iterations \t Success Rate")
    for i in range(len(iterations)):
        print(iterations[i], " \t", successes[i])


def test_rrt_star_success_rate():
    width, height, ob, pr = file_parse.parse_problem("test_cases/robot_env_01.txt", "test_cases/probs_01.txt")
    successes = []
    iterations = []

    robot = Robot(width, height)
    for i in range(10, 201, 10):
        print("Measuring with {} iterations".format(i))
        s = []
        for j in range(20):
            success, tree, path = rrt_star.RRT_star(robot, ob, pr[0][0], pr[0][1], i, radius_around_goal=0.2)
            s.append(1 if success else 0)
        iterations.append(i)
        successes.append(sum(s)/len(s))
    print("Iterations \t Success Rate")
    for i in range(len(iterations)):
        print(iterations[i], " \t", successes[i])


if __name__ == '__main__':
    test_rrt_star()
