def parse_landmarks(id_number):
    landmarks = []
    with open("landmark_data/landmark{}.txt".format(str(id_number)), "r") as lf:
        for i, line in enumerate(lf):
            if i == 0:
                continue
            landmarks.append((int(line.split()[0]), int(line.split()[1])))
    return landmarks
