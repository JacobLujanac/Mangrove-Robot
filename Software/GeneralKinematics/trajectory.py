import sys, os
sys.path.append(os.path.abspath("."))
import numpy as np
from Software.Core.math_utils import *


# Trajectory generator
def gen_trajectory(traj_fn, ease_fn, start_pos, end_pos, steps, leg, move_type = "moveFeet"):
    traj_global = traj_fn(leg, start_pos, end_pos, steps)

    traj_local = convert_traj(move_type, leg, traj_global)

    eased_coords = []
    for i in range(steps):
        t = i / (steps - 1)
        eased_t = ease_fn(t)
        idx = eased_t * (steps - 1)
        low = int(np.floor(idx))
        high = min(low + 1, steps - 1)
        alpha = idx - low
        interp = (1 - alpha) * traj_local[low] + alpha * traj_local[high]
        eased_coords.append(interp)
    return np.array(eased_coords)

def convert_traj(move_type, leg, x, y, psi = None):
    """
    Converts trajectory from global coordinates to those required for the foot to move
    """

    if psi == None:
         psi = leg.robot.yaw
         
    x_rot = np.cos(psi) * x - np.sin(psi) * y 
    y_rot = np.sin(psi) * x + np.cos(psi) * y

    if move_type == "moveHip":
            
            x_converted = leg.fk_xyz()[0] - x_rot #Translate from hip to foot reference frame, and invert trajectory
            y_converted = leg.fk_xyz()[1] - y_rot
            xy_converted = np.hstack(x_converted, y_converted)
    else:
         xy_converted = np.hstack(x_rot, y_rot)

    return xy_converted

def lin_trajectory(leg, start_pos, end_pos, steps):
    
    coord1 = np.linspace(start_pos[0], end_pos[0], steps)
    coord2 = np.linspace(start_pos[1], end_pos[1], steps)

    return coord1, coord2

def rotation_arc_trajectory(leg, start_yaw, end_yaw, steps):


    radius = leg.hip_pos
    theta_hip_CoG = DegMath.arctan2(radius[1],radius[2])
    psi = (np.linspace(start_yaw, end_yaw, steps) + theta_hip_CoG)
    x = (radius[0] * DegMath.cos(psi)) - radius[0]
    y = (radius[1] * DegMath.sin(psi)) - radius[1]

    psi = -1 * psi #Undo body rotation to convert global coords to leg's local coords
    return x, y, psi





