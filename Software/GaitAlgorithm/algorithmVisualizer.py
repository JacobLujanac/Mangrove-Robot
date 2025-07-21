import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
import matplotlib.pyplot as plt
from Software.GaitAlgorithm.visual_utils import *
from Software.GaitAlgorithm.robot_utils import *

tripods = cfg.TRIPOD_LEGS


def visualize_gait(planner_result, use_tripod="both"):
    pose = planner_result["robot_pose"]
    CoG = [pose["x"],pose["y"]]
    hips = planner_result["hip_positions_world"]
    feet = planner_result["foot_positions_world"]
    translation_roms = planner_result["translation_roms"]
    planting_roms = planner_result["planting_roms"]
    root_points = planner_result["roots"]
    tall_root_points = planner_result["tall_roots"]

    if use_tripod == "A":
        legs_to_draw = cfg.TRIPOD_LEGS["A"]
    elif use_tripod == "B":
        legs_to_draw = cfg.TRIPOD_LEGS["B"]
    else:
        legs_to_draw = list(feet.keys())

    hips_subset = {leg: hips[leg] for leg in legs_to_draw}
    feet_subset = {leg: feet[leg] for leg in legs_to_draw}

    # Draw Work Space
    fig, ax = plt.subplots(figsize=(8, 8))

    if root_points:
        rx, ry = zip(*root_points)
        ax.scatter(rx, ry, color='black', marker='^', label='Root', zorder=3)
    if tall_root_points:
        tx, ty = zip(*tall_root_points)
        ax.scatter(tx, ty, color='purple', label='Tall Root')
    
    # Draw Robot
    draw_robot(ax, pose)
    hips_world = hips
    feet_world = feet
    draw_legs(ax, {leg: hips_world[leg] for leg in legs_to_draw}, {leg: feet_world[leg] for leg in legs_to_draw})
    draw_hip_vectors(ax,pose,{leg: CoG for leg in legs_to_draw})
    
    # Draw Base of Support
    draw_bos_triangle(ax, feet_world, legs_to_draw)

    # Draw Planting ROMs
    for leg in legs_to_draw:
        rom = planting_roms[leg]
        draw_rom(ax, rom["center"], rom["ROM"], leg)
        feet_world_dict = np.array(feet_world[leg])
        

        
        
        hips_dict = {leg: CoG}  # Hip at origin
        feet_CoG_dict = {leg: [feet[leg][0]+CoG[0],feet[leg][1]+CoG[1]]}
      
        

    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Robot Pose and Planned Foot Placement")
    ax.legend(loc="upper left", bbox_to_anchor=(1, 1))
    plt.show(block=False)
