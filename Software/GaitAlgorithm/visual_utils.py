import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import numpy as np
from Software.GaitAlgorithm.robot_utils import *
from shapely.plotting import plot_polygon
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.affinity import translate
from matplotlib.patches import Polygon as MplPolygon
import Software.GaitAlgorithm.robot_utils as robot_utils

import Software.Core.config as cfg
fullLegNames = {
    "RF": "Right Front",
    "LF": "Left Front",
    "RM": "Right Middle",
    "LM": "Left Middle",
    "RB": "Right Back",
    "LB": "Left Back"
}

r_min = cfg.r_min
r_max = cfg.r_max
r_clearance = cfg.balance_clearance



def draw_robot(ax, pose):
    x, y = pose["x"], pose["y"]
    w, h = pose["width"], pose["length"]
    yaw = pose["yaw"]


    dx = w / 2
    dy = h / 2
    corners = np.array([
        [-dx, -dy],
        [ dx, -dy],
        [ dx,  dy],
        [-dx,  dy]
    ])

    R = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])

    rotated = (R @ corners.T).T + np.array([x, y])
    robot_body = patches.Polygon(rotated, closed=True, facecolor=cfg.color_dict["Robot"], edgecolor='black', label='Robot', zorder=4)
    ax.add_patch(robot_body)

    circle = plt.Circle((x, y), r_clearance, color='purple', label='Balance Clearance', zorder=5)
    ax.add_patch(circle)

def draw_legs(ax, hips_dict, feet_dict, show_ends = True,alphaVal = 1):
    for name in hips_dict:
        # Get hip and foot positions
        hx, hy = hips_dict[name]
        fx, fy = feet_dict[name]

        if show_ends == True:
            # Draw hip
            ax.plot(hx, hy,
                    color=cfg.color_dict["Robot"],
                    marker='o',
                    markersize=10,
                    markeredgecolor= cfg.color_dict[name],
                    zorder=6)

            # Draw foot
            ax.plot(fx, fy,
                    color=cfg.color_dict["Robot"],
                    marker='X',
                    markersize=10,
                    markeredgecolor=cfg.color_dict[name],
                    zorder=6)

        # Draw line connecting hip to foot
        ax.plot([hx, fx], [hy, fy],
                color= cfg.color_dict[name],
                linewidth=4.5,
                zorder=4,
                alpha=alphaVal,
                )
        ax.plot([hx, fx], [hy, fy],
                color=cfg.color_dict["Robot"],
                linewidth=3,
                zorder=5,
                alpha=alphaVal,
                )

def draw_hip_vectors(ax,robot_pose,start_dict,alphaVal = 1, reverse = False):

    hip_pos = robot_utils.get_hip_positions(robot_pose, robot_pose["tripod_legs"], mode = "Relative")

    for name in start_dict:
        # Get hip and foot positions
        hx, hy = hip_pos[name]
        if reverse == True:
            hx, hy = -hx, -hy       
        ox, oy = start_dict[name]


    
        # Draw hip vectors
        ax.plot([ox, ox+hx], [oy, oy+hy],
                color= cfg.color_dict["Robot"],
                linewidth=4.5,
                zorder=4,
                alpha= alphaVal,
                )
        ax.plot([ox, ox+hx], [oy, oy+hy],
                color=cfg.color_dict[name],
                linewidth=3,
                zorder=5,
                alpha=alphaVal,
                )





def draw_rom(ax, center_pos, rom_geom, leg_name):
    
    color = cfg.color_dict[leg_name]
    

    #Draw outer boundary 
    
    if rom_geom.geom_type == "MultiPolygon":
        polygons = list(rom_geom.geoms)
        
    else:
        polygons = [rom_geom]

    for polygon in polygons :   
        coords = np.array(polygon.exterior.coords)
        patch = MplPolygon(coords, closed=True,
                        facecolor= color, edgecolor = color,
                        alpha=0.3, zorder=1)
        ax.add_patch(patch)

        # Draw interior holes (if any)
    
        for interior in polygon.interiors:
            hole_coords = np.array(interior.coords)
            hole = MplPolygon(hole_coords, closed=True,
                            facecolor='white', edgecolor='white',
                            zorder=4)
            ax.add_patch(hole)
   


def draw_bos_triangle(ax, feet_dict, leg_names):
    """
    Draws a filled black triangle (alpha=0.2) connecting the 3 given feet.
    Also draws 3 solid black lines for the edges (alpha=1).
    """
    assert len(leg_names) == 3, "BoS triangle must be defined by exactly 3 legs."

    # Get triangle vertices
    pts = [feet_dict[leg] for leg in leg_names]
    pts.append(pts[0])  # close the loop for edge drawing

    # Filled triangle (semi-transparent)
    triangle = MplPolygon(pts[:-1], closed=True,
                       facecolor='black', alpha=0.2,
                       zorder = 3)
    ax.add_patch(triangle)

    # Draw edges with solid black lines
    for i in range(3):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        ax.plot([x0, x1], [y0, y1], color='black', linewidth=1.5, alpha=1.0, zorder=4)


def draw_tripod_leg_roms(tripod_feet, root_points):
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))
    for i, (leg_name, foot_pos) in enumerate(tripod_feet.items()):
        ax = axs[i]
        ax.set_title(f"{leg_name} ROM")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_aspect('equal')
        ax.grid(False)
        ax.axhline(0, color='black', linewidth=0.75, linestyle='--')
        ax.axvline(0, color='black', linewidth=0.75, linestyle='--')
        draw_rom(ax, foot_pos, leg_name)
        ax.plot(0, 0, 'ko', label='Hip (origin)', zorder=4)
        ax.plot(foot_pos[0], foot_pos[1], 'x', color='black', markersize=8, label='Foot')
        local_roots = [(rx - foot_pos[0], ry - foot_pos[1]) for (rx, ry) in root_points]
        if local_roots:
            lx, ly = zip(*local_roots)
            ax.scatter(lx, ly, color='black', marker='^', s=10, zorder=3)
        ax.set_xlim(-r_max - 5, r_max + 5)
        ax.set_ylim(-r_max - 5, r_max + 5)
        ax.legend()
    plt.tight_layout()
    plt.show(block=False)

def draw_tripod_leg_roms_from_result(planner_result, tripod_name):
    """
    Draws 3 side-by-side hip ROM plots for the selected tripod,
    using data from planner_result. Includes legs drawn with same styling as global plot.
    """
    tripods = cfg.TRIPOD_LEGS

    leg_names = tripods[tripod_name]
    foot_positions_world = planner_result["foot_positions_world"]
    hip_positions_world = planner_result["hip_positions_world"]
    root_points = planner_result["roots"]
    translation_roms = planner_result["translation_roms"]

    fig, axs = plt.subplots(1, 3, figsize=(18, 6))

    for i, leg_name in enumerate(leg_names):
        ax = axs[i]
        ax.set_title(f"{leg_name} ROM (Relative Displacement)")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.set_aspect('equal')
        ax.grid(False)
        ax.axhline(0, color='black', linewidth=1, linestyle='--')
        ax.axvline(0, color='black', linewidth=1, linestyle='--')

        # Feet Relative to Hips
        foot_hip = np.array(foot_positions_world[leg_name]) - np.array(hip_positions_world[leg_name])
        
        # Hips Relative to World
        hip_world = np.array(hip_positions_world[leg_name])
        
        # ROM centered at foot
        draw_rom(ax, foot_hip, translation_roms[leg_name]["ROM"], leg_name)

        # === Draw Leg === #
        hips_dict = {leg_name: (0, 0)}  # Hip at origin
        feet_dict = {leg_name: tuple(foot_hip)} 
        draw_legs(ax,hips_dict,feet_dict)
       
       # === Draw Roots === #
        all_roots = planner_result["roots"]

        local_roots = [(rx - hip_world[0], ry - hip_world[1]) for (rx, ry) in all_roots]
        local_collidable_roots = planner_result["translation_roms"][leg_name]["roots_inside"]
        if local_roots:
            rx, ry = zip(*local_roots)
            ax.scatter(rx, ry, color='black', alpha = 0.3, marker='^', s=10, zorder=3)

        if local_collidable_roots:
            rcx, rcy = zip(*local_collidable_roots)
            ax.scatter(rcx, rcy, color='black', alpha = 1, marker='^', s=10, zorder=5)

        ax.set_xlim(-r_max - 5, r_max + 5)
        ax.set_ylim(-r_max - 5, r_max + 5)
        ax.legend()

    plt.tight_layout()
    plt.show(block=False)


def draw_viable_translation_zone(planner_result):
    """
    Draws all 3 hip ROMs and their overlap in a single figure.
    Used to visualize the viable translation zone.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from shapely.geometry import mapping
    foot_positions_world = planner_result["foot_positions_world"]
    tripod_legs = planner_result["robot_pose"]["tripod_legs"]
    viable_zone = planner_result["viable_zone"]
    robot_pose = planner_result["robot_pose"]
    translation_roms = planner_result["translation_roms"]

    feet_CoG = {
    leg: np.array(foot_positions_world[leg]) - np.array([robot_pose["x"],robot_pose["y"]])
    for leg in tripod_legs
    }

    hip_positions_world = robot_utils.get_hip_positions(robot_pose, tripod_legs, mode = "Global")
    fig, ax = plt.subplots(figsize=(6, 6))
    feet_hip = {
    leg: np.array(np.array(foot_positions_world[leg]) - np.array(hip_positions_world[leg]))
    for leg in tripod_legs
    }

    # Draw Legs
    for leg in tripod_legs:
        hips_dict = {leg: (0, 0)}  # Hip at origin
        draw_legs(ax,hips_dict,feet_hip, show_ends = False)
        

        # Draw ROMs
        feet = feet_hip[leg]
        rom = translation_roms[leg]["ROM"]
        draw_rom(ax, feet, rom, leg)

        # Draw hip vectors
        draw_hip_vectors(ax,robot_pose,feet_CoG,reverse=True)
        

    if viable_zone.geom_type == "MultiPolygon":
        polygons = list(viable_zone.geoms)
        
    elif not viable_zone.is_empty :
        polygons = [viable_zone]
    #Draw outer boundary
    
        for polygon in polygons:
            coords = np.array(polygon.exterior.coords)
            patch = MplPolygon(coords, closed=True,
                            facecolor= "#28a17f", edgecolor = 'black',
                            alpha=1, zorder=1)

            ax.add_patch(patch)

        # Draw interior holes (if any)
        for polygon in polygons:
            for interior in polygon.interiors:
                hole_coords = np.array(interior.coords)
                hole = MplPolygon(hole_coords, closed=True,
                                facecolor='white', edgecolor='black',
                                zorder=4)
                ax.add_patch(hole)
    # Draw Base of Support Triangle
    draw_bos_triangle(ax, feet_CoG, tripod_legs)

    # Draw origin dot
    ax.plot(0, 0, 'ko', markersize=8, label='Stacked Hips/CoG', zorder=6)
    circle = plt.Circle((0, 0), r_clearance, color='black', zorder=5)
    ax.add_patch(circle)

    ax.set_title("Viable Translation Zone (Overlap of ROMs)")
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_aspect("equal")
    ax.grid(False)
    ax.axhline(0, color='black', linewidth=0.75, linestyle='--')
    ax.axvline(0, color='black', linewidth=0.75, linestyle='--')
    ax.legend()
    plt.tight_layout()
    plt.show(block=False)

