# Servo2040 Side Configurations

LEG_NAMES = ["RB","LB","RM","LM","RF","LF"]


GLOBAL_OFFSETS = {
    "Hip": {
        "RF": 45, "RM": 0, "RB": -45,
        "LF": 135, "LM": 180, "LB": -135
    },
    "Knee": 0,
    "Ankle": 0
}

#Offset from Installation
JOINT_OFFSETS = {
    "Hip": {
        "RF": -5, "RM": -3, "RB": -3,
        "LF": -15, "LM": 0, "LB": -6
    },
    "Knee": {
        "RF": 2, "RM": 5, "RB": -3,
        "LF": 6,"LM": -15, "LB": 0
    },
   "Ankle": {
    "RF": -126, "RM": -95, "RB": -103,
    "LF": -100,"LM": -113, "LB": -100
    }
} 

ROM_LIMITS = {
    "Hip": (-90, 90),
    "Knee": (-90, 90),
    "Ankle": (-135, 135)
}
