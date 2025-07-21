import sys, os, json
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
import Software.GeneralKinematics.kinematics as kin
import Software.Core.Robot as robot
from Software.Core.math_utils import DegMath
from Software.GaitAlgorithm.robot_utils import *
from Software.GaitAlgorithm.algorithmVisualizer import visualize_gait
from Software.GaitAlgorithm.visual_utils import draw_viable_translation_zone, draw_tripod_leg_roms_from_result
import numpy as np
from sympy import symbols, Piecewise, And, solveset, Interval, S, Lt, Union, Min, Max
import matplotlib.pyplot as plt
from shapely.plotting import plot_polygon
# 3D Plotting
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import Polygon, MultiPolygon


tripods = cfg.TRIPOD_LEGS




            
def gait_algorithm(robot, visualize = True):

    # Set Active Tripod Legs
    tripod_legs = robot.tripod_legs[robot.active_tripod]



    # Load Initial Guess on First
  
    initial_guess = {"A":
                        {"RF": polar_to_cartesian(cfg.IDEAL_BOS["F"][0], cfg.IDEAL_BOS["F"][1]),
                        "LM": polar_to_cartesian(cfg.IDEAL_BOS["M"][0], cfg.IDEAL_BOS["M"][1]),
                        "RB": polar_to_cartesian(cfg.IDEAL_BOS["B"][0], cfg.IDEAL_BOS["B"][1])
                        },
                    "B":
                        {"LF": polar_to_cartesian(cfg.IDEAL_BOS["F"][0], 180 - cfg.IDEAL_BOS["F"][1]),
                        "RM": polar_to_cartesian(cfg.IDEAL_BOS["M"][0], 180 - cfg.IDEAL_BOS["M"][1]),
                        "LB": polar_to_cartesian(cfg.IDEAL_BOS["B"][0], 180 - cfg.IDEAL_BOS["B"][1])
                        }
                    }
    
    # Feet Positions Relative to Respective Hips
    rel_feet = {leg_name: initial_guess[robot.active_tripod][leg_name] for leg_name in tripod_legs}

    yaw = robot.yaw
    R = np.array([
            [DegMath.cos(yaw), -DegMath.sin(yaw)],
            [DegMath.sin(yaw),  DegMath.cos(yaw)]
        ])
    # Hips Relative to CoG
    rel_hip = {
        leg: R @ np.array(cfg.HIP_POS[leg])
        for leg in tripod_legs
    }
    # Feet Relative to CoG
    feet_CoG = {
        leg: np.add(rel_feet[leg],rel_hip[leg])
        for leg in tripod_legs
        }    
                  
        
    psi_range = np.arange(-180,180,5)
    viable_zone = {}
    base_of_support = get_bos(feet_CoG)
    yaw_initial = robot.yaw
    for psi in psi_range:
        robot.yaw = psi
        # === ROBOT STATE ===
        robot_pose = {
            "x": robot.x_global,
            "y": robot.y_global,
            "yaw": robot.yaw,
            "width": cfg.body_width,
            "length": cfg.body_length,
            "balance_clearance": cfg.balance_clearance,
            "tripod_legs": tripod_legs
        }


        # === FOOT POSITIONS (INITIAL GUESS) === #


        feet = {
            leg: np.add(rel_feet[leg],np.array(cfg.HIP_POS[leg]))
            for leg in tripod_legs
        }

        # === LOAD ROOTS FROM MAP ===

        # Get absolute path to the directory of the current script
        script_dir = os.path.dirname(__file__)
        file_path = os.path.join(script_dir, "map_data.json")

        with open(file_path, "r") as f:
                map_data = json.load(f)

        root_points = [tuple(p) for p in map_data["root_points"]]
        target_points = [tuple(p) for p in map_data["target_points"]]
        

        # === Global Hip and Feet Positions === #
        hip_positions_world = get_hip_positions(robot_pose, tripod_legs)
        
        foot_positions_world = get_feet_world_positions(feet,hip_positions_world,robot_pose)


        # === Viable Translation Zone === #
        yaw = robot_pose["yaw"]
        R = np.array([
                [DegMath.cos(yaw), -DegMath.sin(yaw)],
                [DegMath.sin(yaw),  DegMath.cos(yaw)]
            ])
        # Hips Relative to CoG
        rel_hip_positions = {
            leg: R @ np.array(cfg.HIP_POS[leg])
            for leg in tripod_legs
        }

        # Feet Relative to Hips
        feet_hip = {
            leg: np.array(np.array(feet[leg]) - np.array(rel_hip_positions[leg]))
            for leg in tripod_legs
            }
        

        planting_roms = {}
        for leg in tripod_legs:
            hip_pos = hip_positions_world[leg]

            # Filter to only roots within planting ROM
            roots_inside = get_roots_in_bounds(hip_pos, root_points)
            
            planting_roms[leg] = {
                "center": hip_pos,
                "ROM": get_rom_region(hip_pos,roots_inside),
                "roots_inside": roots_inside
            }

        translation_roms = {}
        rom_regions = []
        for leg in tripod_legs:
            foot_pos = foot_positions_world[leg]
            hip_pos = hip_positions_world[leg]

            # Filter to only roots that can collide with l_subRoot_max
            filtered_roots = get_roots_in_bounds(foot_pos,root_points, 0, cfg.l_subRoot_max)

            # Convert to Local Frame (Hip at Origin)
            collidable_roots = [(rx - hip_pos[0], ry - hip_pos[1]) for (rx, ry) in filtered_roots]
            
            # Compute translation ROMs for each hip  

            rom_region = get_rom_region(feet_hip[leg], collidable_roots, type = "translate", center = "off")
            rom_regions.append(rom_region)
            translation_roms[leg] = {
                "ROM":rom_region,
                "roots_inside": collidable_roots   
            }


        # Compute overall viable translation zone
        viable_zone_uncropped = compute_viable_translation_zone(rom_regions)

        # Only include zone within base of support
        viable_zone[psi] = viable_zone_uncropped.intersection(base_of_support)

    robot.yaw = yaw_initial
    start = (0,0,robot.yaw)

    
    
    
    # Determine Closest Target
    target_CoG = np.array(target_points) - np.array([robot_pose["x"],robot_pose["y"]])
    closest_target_indx = np.argmin(np.linalg.norm(target_CoG))
    closest_target = target_CoG[closest_target_indx]
    target_direction = np.array([closest_target[0], closest_target[1],0])
    print("target_direction:",target_direction)
    # Find Continous Path that Travels Furtherst Distance Towards Target
    
    path = greedy_bfs(start,target_direction,viable_zone)
    print(path[0])

    # Plot Viable Translation Zone for each Yaw Value
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot Path
    ax.plot(path[:, 0], path[:, 1], path[:, 2], color='red', linewidth=2)

    # Plot Target Direction
    ax.quiver(*start,*target_direction,color = 'green',normalize = "True", length = 5,arrow_length_ratio=0.5)
    for psi, zone in viable_zone.items():
        if isinstance(zone, Polygon):
            polys = [zone]
        elif isinstance(zone, MultiPolygon):
            polys = list(zone.geoms)
        else:
            continue  # skip if invalid

        for poly in polys:
            # Extract exterior coordinates
            x, y = poly.exterior.xy
            z = np.full_like(x, psi)

            
            verts = [list(zip(x, y, z))]

            if verts[0]:
                collection = Poly3DCollection(verts, alpha=0.1)
                ax.add_collection3d(collection)

    ax.set_xlabel(r'$\Delta$x')
    ax.set_ylabel(r'$\Delta$y')
    ax.set_zlabel(r'$\Psi$')
    plt.show()
    # === PACK RESULT === #
    planner_result = {
        "robot_pose": robot_pose,
        "foot_positions_world": foot_positions_world,
        "hip_positions_world": hip_positions_world,
        "translation_roms": translation_roms,
        "planting_roms": planting_roms,
        "roots": root_points,
        "viable_zone": viable_zone
    }

    if visualize == True:

        visualize_gait(planner_result, use_tripod= robot.active_tripod)
        draw_tripod_leg_roms_from_result(planner_result, tripod_name= robot.active_tripod)

        # === Visualize the viable translation zone ===
        draw_viable_translation_zone(planner_result)
        plt.show()
    return planner_result