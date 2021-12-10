import math
import numpy as np

class Robot:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.translation = (0, 0)
        self.rotation = 0
        self.robot_origin_coords = [(-self.width / 2, -self.height / 2), (-self.width / 2, self.height / 2),
                                    (self.width / 2, self.height / 2), (self.width / 2, -self.height / 2)]
        self.robot_rotated_coords = self.robot_origin_coords
        self.robot_current_state = self.robot_origin_coords
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.state_vector = [0, 0, 0]
        self.velocity_vector = [0, 0]

    def set_pose(self, pose):
        self.translation = (pose[0], pose[1])
        self.rotation = pose[2]

    def transform(self):
        self.rotate(self.rotation)
        self.robot_current_state = self.translate(self.translation)
        return self.robot_current_state

    def translate(self, target: tuple):
        new_x, new_y = target
        new_coords = []
        for c in self.robot_rotated_coords:
            new_coords.append((c[0] + new_x, c[1] + new_y))
        return new_coords

    def rotate(self, angle):
        new_coords = []
        for c in self.robot_origin_coords:
            ox, oy = (0, 0)
            px, py = c[0], c[1]

            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
            new_coords.append((qx, qy))
        self.robot_rotated_coords = new_coords
        return new_coords

    def kinematics(self, state, control):
        q, u = state, control
        k_val = []
        k_val.append((control[0] * np.cos(state[2]) + (np.pi / 2)))
        k_val.append((control[0] * np.sin(state[2]) + (np.pi / 2)))
        k_val.append(control[1])
        return k_val

    def propagate(self, state, controls, durations, dt):
        curr = state
        seq = [state]
        for i in range(len(controls)):
            k = self.kinematics(curr, controls[i])
            k_0 = k[0] * durations[i] * dt
            k_1 = k[1] * durations[i] * dt
            k_2 = k[2] * durations[i] * dt
            new_state = (curr[0] + k_0, curr[1] + k_1, curr[2] + k_2)
            seq.append(new_state)
            curr = new_state
        self.state_vector = curr
        return seq

