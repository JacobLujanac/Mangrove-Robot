from pi.core.leg import Leg

class Robot:
    def __init__(self, z_root):
        self.z_root = z_root
        self.legs = {}

        self.joint_ROM_limits = {
            "Hip": (-90, 90),
            "Knee": (-90, 135),
            "Ankle": (0, 135),
        }

        self.limb_lengths = {
            "coxa": 5.566,
            "femur": 8.679,
            "tibia": 7.042
        }

        # Define servo mappings for each leg
        leg_configs = {
            "RF": {"hip": "L1_hip", "knee": "L1_knee", "ankle": "L1_ankle"},
            "RM": {"hip": "L2_hip", "knee": "L2_knee", "ankle": "L2_ankle"},
            "RB": {"hip": "L3_hip", "knee": "L3_knee", "ankle": "L3_ankle"},
            "LF": {"hip": "L4_hip", "knee": "L4_knee", "ankle": "L4_ankle"},
            "LM": {"hip": "L5_hip", "knee": "L5_knee", "ankle": "L5_ankle"},
            "LB": {"hip": "L6_hip", "knee": "L6_knee", "ankle": "L6_ankle"},
        }

        for name, servos in leg_configs.items():
            self.legs[name] = Leg(name, servos, self.limb_lengths, self.joint_ROM_limits)

    def get_leg(self, name):
        return self.legs[name]

    def get_payload(self):
        """
        Returns a dict of all {servo_id: angle} for the whole robot
        """
        combined = {}
        for leg in self.legs.values():
            combined.update(leg.get_servo_payload())
        return combined
