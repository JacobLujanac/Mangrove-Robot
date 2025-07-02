## Configuration Parameters

steps = 100

# Environment
z_root= 7 #[cm] (1 Peg )
z_bodyClearance = 6 #[cm]

# Robot Specs
body_height = 5 #[cm]
body_width = 13 # [cm]
body_length = 18.679 # [cm]

# === Body relative positions === #
front = body_length / 2
back = -body_length /2 
left = -body_width / 2
right = body_width / 2
middle = 0
center = 0


    

# ROM
joint_ROM_limits = {
            "Hip": (-135, 135),
            "Knee": (-135, 135),
            "Ankle": (-135, 135),
        }

# Limb Lengths [cm]
limb_lengths = {
            "coxa": 5.566,
            "femur": 8.679,
            "tibia": 12.979
        }


LEG_NAMES = ["RF","RM","RB","LF","LM","LB"]

TRIPOD_LEGS = {
            "A": ["RF", "LM", "RB"],
            "B": ["LF", "RM", "LB"]
            }


HIP_OFFSETS = {
    "RF": 45, "RM": 0, "RB": -45,
        "LF": 135, "LM": 180, "LB": -135}

# Distance from CoG to Hips [cm]

HIP_POS = { #Origin: Robot's Center of Gravity
    "RF": [right, front],
    "LF":[left, front],
    "RM":[right, middle],
    "LM":[left, middle],
    "RB":[right, back],
    "LB":[left, back],
}

r_min = 3.0 #cm
r_max = 18.0 #cm

l_subRootMax = 7.5 #cm

balance_clearance = 2 #cm

color_dict = {
    "Target": "#ffdc00",
    "Robot": "#e3e3e3",
    "RF": "#da2929",
    "LF": "#da2929",
    "RM": "#f8e823",
    "LM": "#f8e823",
    "RB": "#3c55bc",
    "LB": "#3c55bc"
}