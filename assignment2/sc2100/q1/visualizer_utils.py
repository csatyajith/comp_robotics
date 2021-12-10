def euclidean_dist(p1, p2):
    return (((p1[0] - p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2)) ** 0.5


def discretize_points_for_animation(path):
    discretization_const = 0.05
    new_path = []
    for i in range(len(path) - 1):
        point1 = path[i]
        point2 = path[i + 1]
        i = point1
        new_x, new_y = point1
        while True:
            len_ab = euclidean_dist((new_x, new_y), point2)
            len_ratio = discretization_const / len_ab
            new_x = round((1 - len_ratio) * i[0] + len_ratio * point2[0], 3)
            new_y = round((1 - len_ratio) * i[1] + len_ratio * point2[1], 3)
            if euclidean_dist(point1, (new_x, new_y)) < euclidean_dist(point1, point2):
                i = new_x, new_y
                new_path.append(i)
            else:
                new_path.append(point2)
                break
    return new_path
