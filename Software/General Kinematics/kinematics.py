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

# rz FK/IK
def fk_rz(self, leg):
    theta_knee = leg.theta_knee
    theta_ankle = leg.theta_ankle
    l_femur = self.l_femur
    l_tibia = self.l_tibia
    r = (l_femur * DegMath.cos(theta_knee) + l_tibia * DegMath.cos(theta_knee + theta_ankle))
    z = (l_femur * DegMath.sin(theta_knee) + l_tibia * DegMath.sin(theta_knee + theta_ankle))
    return r, z

def ik_rz(self, rz_foot):
    r = rz_foot[:, 0]
    z = rz_foot[:, 1]
    l_femur = self.l_femur
    l_tibia = self.l_tibia
    d = np.sqrt(r**2 + z**2)
    D = (d**2 - l_femur**2 - l_tibia**2) / (2 * l_femur * l_tibia)
    if abs(D) > 1.0:
        raise ValueError("Target point is out of reach.")
    theta_ankle_options = [DegMath.arccos(D), -DegMath.arccos(D)]
    solutions = []
    for theta_ankle in theta_ankle_options:
        k1 = l_femur + l_tibia * DegMath.cos(theta_ankle)
        k2 = l_tibia * DegMath.sin(theta_ankle)
        theta_knee = DegMath.arctan2(z, r) - DegMath.arctan2(k2, k1)
        solutions.append([theta_knee, theta_ankle])
    return np.array(solutions)

# xy FK

def fk_xy(self, leg):
    l_leg, _ = self.fk_rz(leg)
    x_hip, y_hip = leg.q_hip
    theta_hip = leg.theta_hip
    x = x_hip + l_leg * DegMath.cos(theta_hip)
    y = y_hip + l_leg * DegMath.sin(theta_hip)
    return x, y

# xyz IK
def ik_xyz(self, xyz_foot, leg):
    x = xyz_foot[:, 0]
    y = xyz_foot[:, 1]
    z = xyz_foot[:, 2]
    x_hip = leg.q_hip[0]
    y_hip = leg.q_hip[1]
    theta_hip = DegMath.arctan2((y - y_hip), (x - x_hip))
    r = np.hypot(x, y)
    theta_knee, theta_ankle = self.ik_rz(np.column_stack((r, z)))
    return np.stack((theta_hip, theta_knee, theta_ankle), axis=1)


### angle_clamp.py
def clamp_angle(angle, joint_index, leg_obj):
    min_angle, max_angle = leg_obj.joint_rom[joint_index]
    return max(min(angle, max_angle), min_angle)
