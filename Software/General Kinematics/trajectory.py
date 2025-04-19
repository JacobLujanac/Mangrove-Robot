import numpy as np
from .math_utils import ease_in_out, ease_out_quad, ease_in_quad, ease_sin

# Trajectory generator
def gen_trajectory(traj_fn, ease_fn, start_pos, end_pos, steps, footPlanted=False):
    if footPlanted:
        end_pos = -end_pos
    traj = traj_fn(start_pos, end_pos, steps)
    eased_coords = []
    for i in range(steps):
        t = i / (steps - 1)
        eased_t = ease_fn(t)
        idx = eased_t * (steps - 1)
        low = int(np.floor(idx))
        high = min(low + 1, steps - 1)
        alpha = idx - low
        interp = (1 - alpha) * traj[low] + alpha * traj[high]
        eased_coords.append(interp)
    return np.array(eased_coords)

def lin_trajectory(start_pos, end_pos, steps):
    coord1 = np.linspace(start_pos[0], end_pos[0], steps)
    coord2 = np.linspace(start_pos[1], end_pos[1], steps)
    return np.stack((coord1, coord2), axis=-1)

