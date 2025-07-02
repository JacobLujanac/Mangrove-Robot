# Servo2040 Side Configurations

LEG_NAMES = ["RB","LB","RM","LM","RF","LF"]  # Expand later

JOINT_OFFSETS = {
    "Hip": {
        "RF": 45, "RM": 0, "RB": -45,
        "LF": 135, "LM": 180, "LB": -135
    },
    "Knee": 0,
    "Ankle": 0
}

ROM_LIMITS = {
    "Hip": (-90, 90),
    "Knee": (-90, 90),
    "Ankle": (-135, 135)
}