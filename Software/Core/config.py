## Configuration Parameters

steps = 100

# Environment
z_root= 7 # Avg Root Height[cm] (1 Peg)
z_bodyClearance = 6 #[cm]
d_r = 1 # Root Diameter [cm] 

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

r_min = 5 #cm
r_max = 15 #cm

# l_subRoot extrema (spans from entry 1 to 2)
l_subRoot_r_min = [r_min, 10.08] #cm
l_subRoot_r_max = [r_max, 14.22] #cm

l_subRoot_max = l_subRoot_r_min[1] - l_subRoot_r_min[0] #cm
    

# ROM
JOINT_ROM_LIMITS = {
            "Hip": (-135, 135),
            "Knee": (-135, 135),
            "Ankle": (-135, 135),
        }

# Limb Lengths [cm]
LIMB_PARAMETERS = {
            "l_coxa": 5.566,
            "l_femur": 8.679,
            "l_tibia_eff": 12.979, # Line from ankle to foot
            "l_tibia_top": 6.573,
            "theta_tibia_bend": -52.53, # 
            "theta_tibia_top": 28.87, # Angle between top segment of tibia and tibia_eff
            "l_tibia_bottom": 7.886

        }




LEG_NAMES = ["RF","RM","RB","LF","LM","LB"]

TRIPOD_LEGS = {
            "A": ["RF", "LM", "RB"],
            "B": ["LF", "RM", "LB"]
            }

IDEAL_BOS = {"F": [0.8 * r_max, 110],
             "M": [0.4 * r_max, 165],
             "B": [0.3 * r_max, 45]} #[r,theta]


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



balance_clearance = 1 #cm

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