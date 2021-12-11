import file_parse
import visualizer
from robot import Robot


def test_parse_visualizer():
    landmarks = file_parse.parse_landmarks("0")
    robot = Robot(2, 3)
    visualizer.visualize_problem(robot, [], (5, 5, 1.57), (95, 95, 1.57), landmarks)


if __name__ == '__main__':
    test_parse_visualizer()
