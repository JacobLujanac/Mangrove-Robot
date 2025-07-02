import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
from robot_utils import *
import json
from robot_utils import compute_viable_translation_zone

tripods = cfg.TRIPOD_LEGS

def gait_algorithm(self,robot):

    # === ROBOT STATE IN CENTIMETERS ===
    robot_pose = {
        "x": robot.x_global,
        "y": robot.y_global,
        "yaw": robot.yaw,
        "width": cfg.robot_width,
        "height": cfg.robot_height,
        "balance_clearance": cfg.balance_clearance
    }

   
    selected_tripod = robot.active_tripod
    tripod_legs = tripods[selected_tripod]

    # === FOOT POSITIONS (INITIAL GUESS) ===

    initial_guess = [polar_to_cartesian(cfg.r_max * 0.8, 80),
                     ]
    rel_feet = {
        tripod_legs[0]: ,
        "RM": polar_to_cartesian(cfg.r_max * 0.4, 12),
        "LB": polar_to_cartesian(cfg.r_max * 0.3, 135)
    }

    


    feet = {
        leg: np.add(rel_feet[leg],np.array(cfg.HIP_POS[leg]))
        for leg in tripod_legs
    }

    # === LOAD ROOTS FROM MAP ===
    with open("map_data.json", "r") as f:
        map_data = json.load(f)

    root_points = [tuple(p) for p in map_data["root_points"]]
    tall_root_points = [tuple(p) for p in map_data["tallRoot_points"]]

    # === Global Hip and Feet Positions ===
    hip_positions_world = get_hip_positions(robot_pose)
    foot_positions_world = get_feet_world_positions(feet,hip_positions_world,robot_pose)

    # === Viable Translation Zone === #
    yaw = robot_pose["yaw"]
    R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])
    rel_hip_positions = {
        leg: R @ np.array(cfg.HIP_POS[leg])
        for leg in tripod_legs
    }
    feet_hip = {
        leg: np.array(np.array(feet[leg]) - np.array(rel_hip_positions[leg]))
        for leg in tripod_legs
        }
    viable_zone = compute_viable_translation_zone(feet_hip, cfg.r_min, cfg.r_max, tripod_legs)

    planting_roms = {}
    for leg in tripod_legs:
        hip_pos = hip_positions_world[leg]
        roots_inside = get_roots_inside_rom(hip_pos, root_points, cfg.r_min, cfg.r_max)
        planting_roms[leg] = {
            "center": hip_pos,
            "r_min": cfg.r_min,
            "r_max": cfg.r_max,
            "roots_inside": roots_inside
        }
    translation_roms = {}

    for leg in tripod_legs:
        foot_pos = foot_positions_world[leg]
        hip_pos = hip_positions_world[leg]

        # Convert global roots to local (hip-at-origin) frame
        roots_local = [(rx - hip_pos[0], ry - hip_pos[1]) for (rx, ry) in root_points]

        # Filter to only roots that can collide with l_subRootMax
        filtered_roots = filter_local_roots_within_collision_zone(foot_pos, roots_local, cfg.l_subRootMax)

        translation_roms[leg] = {
            "center": foot_pos,
            "r_min": cfg.r_min,
            "r_max": cfg.r_max,
            "roots_inside": filtered_roots
        }

    # === PACK RESULT ===
    planner_result = {
        "robot_pose": robot_pose,
        "foot_positions": feet,
        "hip_positions": hip_positions_world,
        "translation_roms": translation_roms,
        "planting_roms": planting_roms,
        "tripod": selected_tripod,
        "roots": root_points,
        "tall_roots": tall_root_points,
        "viable_zone": viable_zone
    }
return