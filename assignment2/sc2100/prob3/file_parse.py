def get_all_xy_coords_from_list(coords_list):
    i = 0
    coords = []
    while i < len(coords_list):
        coords.append((float(coords_list[i]), float(coords_list[i + 1])))
        i += 2
    return coords


def get_all_xytheta_coords_from_list(coords_list):
    i = 0
    coords = []
    while i < len(coords_list):
        coords.append((float(coords_list[i]), float(coords_list[i + 1]),  float(coords_list[i + 2])))
        i += 3
    return coords


def parse_problem(world_file, problem_file):
    with open(world_file) as wf:
        width, height = get_all_xy_coords_from_list(wf.readline().split(" "))[0]
        obstacles = []
        for line in wf:
            vals = get_all_xy_coords_from_list(line.split(" "))
            obstacles.append(vals)

    with open(problem_file) as pf:
        problems = []
        for line in pf:
            vals = get_all_xytheta_coords_from_list(line.split(" "))
            problems.append(vals)
    return width, height, obstacles, problems
