from servo import Servo, servo2040

JOINT_OFFSETS = {
    "Hip": {
        "RF": 45, "RM": 0, "RB": -45,
        "LF": 135, "LM": 180, "LB": -135
    },
    "Knee": -90,
    "Ankle": 0
}

ROM_LIMITS = {
    "Hip": (-90, 90),
    "Knee": (-90, 135),
    "Ankle": (0, 135)
}

class Motor:
    def __init__(self, index, joint_type, leg_name):
        self.joint_type = joint_type
        self.leg_name = leg_name

        self.offset = (
            JOINT_OFFSETS[joint_type][leg_name]
            if joint_type == "Hip" else JOINT_OFFSETS[joint_type]
        )
        self.rom_min, self.rom_max = ROM_LIMITS[joint_type]

        self.servo = Servo(getattr(servo2040, f"SERVO_{index + 1}"))

        # Setup calibration
        cal = self.servo.calibration()
        cal.first_value(-135)
        cal.last_value(135)
        cal.limit_to_calibration(True, True)
        self.servo.calibration(cal)

    def set_deg(self, deg):
        deg_adj = deg - self.offset
        deg_adj = max(self.rom_min, min(self.rom_max, deg_adj))
        self.servo.value(deg_adj)
