### kinematics.py
import numpy as np
from .math_utils import DegMath

# Solution filter
def closest_solution(all_solutions, initialAngles):
    n_steps, n_options, n_joints = all_solutions.shape
    chosen = []
    options = all_solutions[0]
    distances = np.abs(options[:, 0] - initialAngles[0])
    best_idx = np.argmin(distances)
    selected = options[best_idx]
    chosen.append(selected)
    for i in range(1, n_steps):
        prev = chosen[-1]
        options = all_solutions[i]
        distances = np.linalg.norm(options - prev, axis=1)
        best_idx = np.argmin(distances)
        chosen.append(options[best_idx])
    return np.array(chosen)



### angle_clamp.py
def clamp_angle(angle, joint_index, leg_obj):
    min_angle, max_angle = leg_obj.joint_rom[joint_index]
    return max(min(angle, max_angle), min_angle)
