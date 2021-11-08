from environment_r import Robot


def on_segment(p, q, r):
    if ((q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and
            (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
        return True
    return False


def orientation(p, q, r):
    val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
    if val > 0:
        return 1
    elif val < 0:
        return 2
    else:
        return 0


def line_segments_intersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if (o1 != o2) and (o3 != o4):
        return True

    if (o1 == 0) and on_segment(p1, p2, q1):
        return True

    if (o2 == 0) and on_segment(p1, q2, q1):
        return True
    
    if (o3 == 0) and on_segment(p2, p1, q2):
        return True

    if (o4 == 0) and on_segment(p2, q1, q2):
        return True

    return False


def line_intersects_polygon(p1, p2, obstacles):
    for o in obstacles:
        n = len(o)
        for i in range(n):
            j = (i+1) % n
            if line_segments_intersect(p1, p2, o[i], o[j]):
                return True
    return False


def isCollisionFree(robot, point, obstacles):
    temp_robot = Robot(robot.width, robot.height)
    temp_robot.set_pose(point)
    temp_robot.transform()
    n = len(temp_robot.robot_current_state)
    for i in range(n):
        j = (i + 1) % n
        p1 = temp_robot.robot_current_state[i]
        p2 = temp_robot.robot_current_state[j]
        if p1[0] < 0 or p1[0] > 10 or p1[1] < 0 or p1[1] > 10 or p2[0] < 0 or p2[0] > 10 or p2[1] < 0 or p2[1] > 10:
            return False
        if line_intersects_polygon(p1, p2, obstacles):
            return False
    return True
