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
