from pi.trajectory.path_generators import lin_trajectory, gen_trajectory
from pi.trajectory.easing import ease_out_quad, ease_in_quad
from pi.core.ik_utils import fk_rz, ik_rz, closest_solution

class Leg:
    def __init__(self, name, servo_ids, limb_lengths, rom_limits):
        self.name = name
        self.servo_ids = servo_ids
        self.limb_lengths = limb_lengths
        self.rom_limits = rom_limits

        self.joint_angles = {"hip": 0, "knee": 0, "ankle": 0}
        self.steps = 20  # default step resolution

    def get_joint_trajectory(self, traj_fn, ease_fn, start, end, space="rz"):
        """
        Generates a joint angle trajectory from a Cartesian path.
        space: "rz" or "xy"
        """
        coords = gen_trajectory(traj_fn, ease_fn, start, end, self.steps)

        if space == "rz":
            solutions = ik_rz(coords, self)
        else:
            solutions = ik_xyz(coords, self)

        return closest_solution(solutions, list(self.joint_angles.values()))

    def foot_lower(self):
        r, z = fk_rz(self)
        return self.get_joint_trajectory(
            lin_trajectory, ease_out_quad,
            start=[r, z],
            end=[r, 0],
            space="rz"
        )

    def foot_raise(self, z_clearance):
        r, z = fk_rz(self)
        return self.get_joint_trajectory(
            lin_trajectory, ease_in_quad,
            start=[r, z],
            end=[r, z_clearance],
            space="rz"
        )

def hip_swing_planted(self, end_pos, traj_fn=lin_trajectory, ease_fn=ease_out_quad):
    """
    Moves the hip joint in the xy-plane while keeping the foot planted.

    Parameters:
        end_pos (array-like): [x, y] target foot location in the xy-plane
        traj_fn (callable): trajectory function (default: linear)
        ease_fn (callable): easing function (default: ease_out_quad)

    Returns:
        ndarray: joint angles [theta_hip, theta_knee, theta_ankle] for each step
    """
    from brain.trajectory import gen_trajectory  # optional: import at top of file
    from brain.kinematics import ik_xyz, closest_solution

    # Current foot position
    start_pos = self.fk_xy()

    # Generate Cartesian trajectory
    steps = self.steps
    xy_traj = gen_trajectory(traj_fn, ease_fn, start_pos, end_pos, steps)

    # Solve for joint angles at each step
    all_joint_solutions = []
    for coord in xy_traj:
        sol = self.robot.ik_xyz(np.array([coord]), self)
        all_joint_solutions.append(sol)

    all_joint_solutions = np.array(all_joint_solutions)  # (n_steps, 1, 3)

    # Initial angles as reference for smoothest solution
    initial_angles = [self.theta_hip, self.theta_knee, self.theta_ankle]

    return closest_solution(all_joint_solutions, initial_angles)

