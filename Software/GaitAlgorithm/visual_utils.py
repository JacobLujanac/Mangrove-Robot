# visual_utils.py
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import numpy as np
import drawConfig as cfg
from robot_utils import *
from shapely.plotting import plot_polygon
from shapely.geometry import Polygon as ShapelyPolygon
from matplotlib.patches import Polygon as MplPolygon
from GaitAlgorithm import robot_pose, tripod_legs
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as config
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

yaw = robot_pose["yaw"]
R = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])
rel_hip_positions = {
    leg: R @ np.array(cfg.relative_hip_positions[leg])
    for leg in tripod_legs
}
def draw_robot(ax, pose):
    x, y = pose["x"], pose["y"]
    w, h = pose["width"], pose["height"]
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

def draw_legs(ax, hips_dict, feet_dict, ends = True,alphaVal = 1):
    for name in hips_dict:
        # Get hip and foot positions
        hx, hy = hips_dict[name]
        fx, fy = feet_dict[name]

        if ends == True:
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
                zorder=5,
                alpha=alphaVal,
                )
        ax.plot([hx, fx], [hy, fy],
                color=cfg.color_dict["Robot"],
                linewidth=3,
                zorder=4,
                alpha=alphaVal,
                )

def draw_hip_vectors(ax,start_dict,alphaVal = 1, reverse = False):
    for name in start_dict:
        # Get hip and foot positions
        hx, hy = rel_hip_positions[name]
        if reverse == True:
            hx, hy = -hx, -hy       
        ox, oy = start_dict[name]


    
        # Draw hip vectors
        ax.plot([ox, ox+hx], [oy, oy+hy],
                color= cfg.color_dict["Robot"],
                linewidth=4.5,
                zorder=7,
                alpha= alphaVal,
                )
        ax.plot([ox, ox+hx], [oy, oy+hy],
                color=cfg.color_dict[name],
                linewidth=3,
                zorder=7,
                alpha=alphaVal,
                )





def draw_rom(ax, center_pos, leg_name):
    x, y = center_pos
    color = cfg.color_dict[leg_name]
    outer = Circle((x, y), r_max, color=color, alpha=0.3, zorder=1)
    inner = Circle((x, y), r_min, color='white', zorder=2)
    ax.add_patch(outer)
    ax.add_patch(inner)


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

def draw_tripod_leg_roms_from_result(planner_result, tripod_name="A"):
    """
    Draws 3 side-by-side hip ROM plots for the selected tripod,
    using data from planner_result. Includes legs drawn with same styling as global plot.
    """
    tripods = {
        "A": ["LF", "RM", "LB"],
        "B": ["RF", "LM", "RB"]
    }

    leg_names = tripods[tripod_name]
    foot_positions = planner_result["foot_positions"]
    root_points = planner_result["roots"]

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

        # Local transform
        foot_pos = np.array(foot_positions[leg_name])
        

        # ROM centered at foot
        draw_rom(ax, foot_pos, leg_name)

        # === Draw Leg ===
        hips_dict = {leg_name: (0, 0)}  # Hip at origin
        feet_dict = {leg_name: tuple(foot_pos)} 
        draw_legs(ax,hips_dict,feet_dict)
       

        # Roots transformed into local frame
        local_roots = [(rx - foot_pos[0], ry - foot_pos[1]) for (rx, ry) in root_points]
        if local_roots:
            lx, ly = zip(*local_roots)
            #ax.scatter(lx, ly, color='black', marker='^', s=10, zorder=3)

        ax.set_xlim(-r_max - 5, r_max + 5)
        ax.set_ylim(-r_max - 5, r_max + 5)
        #ax.legend()

    plt.tight_layout()
    plt.show(block=False)


def draw_viable_translation_zone(foot_positions, tripod_legs, viable_zone, robot_pose):
    """
    Draws all 3 hip ROMs and their overlap in a single figure.
    Used to visualize the viable translation zone.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from shapely.geometry import mapping

    fig, ax = plt.subplots(figsize=(6, 6))
    feet_hip = {
    leg: np.array(np.array(foot_positions[leg]) - np.array(rel_hip_positions[leg]))
    for leg in tripod_legs
    }

    # Draw Legs
    for leg in tripod_legs:
        hips_dict = {leg: (0, 0)}  # Hip at origin
        draw_legs(ax,hips_dict,feet_hip, ends = False)
        

        # Draw ROMs
        feet = feet_hip[leg]
        draw_rom(ax, feet, leg)

        # Draw hip vectors
        draw_hip_vectors(ax,foot_positions,reverse=True)
        


    #Draw outer boundary
    if not viable_zone.is_empty:
        coords = np.array(viable_zone.exterior.coords)
        patch = MplPolygon(coords, closed=True,
                           facecolor= '#f9f9f9', edgecolor = 'black',
                           alpha=1, zorder=1)

        ax.add_patch(patch)

        # Draw interior holes (if any)
        for interior in viable_zone.interiors:
            hole_coords = np.array(interior.coords)
            hole = MplPolygon(hole_coords, closed=True,
                              facecolor='white', edgecolor='black',
                              zorder=4)
            ax.add_patch(hole)
    # Draw Base of Support Triangle
    feet_world = get_feet_world_positions(foot_positions,rel_hip_positions,robot_pose)
    draw_bos_triangle(ax, foot_positions, tripod_legs)

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

