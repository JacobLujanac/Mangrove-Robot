import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
import numpy as np
import math
from shapely.geometry import Point
from shapely.ops import unary_union


# Robot specific
def get_rotation_matrix(yaw):
    return np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw),  np.cos(yaw)]
    ])

def get_hip_positions(pose,mode = "Global"):
    """Calculate world positions of each hip based on robot pose."""
    R = get_rotation_matrix(pose["yaw"])
    center = np.array([pose["x"], pose["y"]])
    hips_world = {}
    for leg_name, rel in cfg.HIP_POS.items():
        rel_vec = np.array(rel)
        world_pos = R @ rel_vec
        if mode == "Global": world_pos += center
        hips_world[leg_name] = tuple(world_pos)
    return hips_world

def get_feet_world_positions(feet_rel_to_cog, hip_positions, robot_pose):
    R = get_rotation_matrix(robot_pose["yaw"])
    feet_world = {}
    CoG = np.array([robot_pose["x"],robot_pose["y"]])
    for leg, foot_local in feet_rel_to_cog.items():
        
        foot_offset = np.array(foot_local)
        foot_world = foot_offset + CoG 
        feet_world[leg] = tuple(foot_world)
    return feet_world
   

def get_roots_inside_rom(center_pos, root_points, r_min, r_max):
    """Returns a list of root coordinates within ROM Bounds."""
    roots_inside = []
    for root in root_points:
        dist = np.linalg.norm(np.array(root) - np.array(center_pos))
        if r_min <= dist <= r_max:
            roots_inside.append(root)
    return roots_inside

def filter_local_roots_within_collision_zone(foot_pos, root_points_local, radius):
    """
    Filters out roots that are farther than `radius` from the origin (foot).
    Operates in local frame centered at foot.
    """
    return [root for root in root_points_local if np.linalg.norm(root) <= radius]

def get_rom_region(center_pos, r_min, r_max, resolution=64):
    """
    Returns a Shapely annulus polygon representing ROM bounds
    """
    center_point = Point(center_pos)
    outer = center_point.buffer(r_max, resolution=resolution)
    inner = center_point.buffer(r_min, resolution=resolution)
    return outer.difference(inner)

def compute_viable_translation_zone(foot_positions, r_min, r_max, leg_names):
    """
    Computes the overlap of ROMs (viable body translation region)
    from the hips of the 3 tripod legs.
    """
    rom_regions = [get_rom_region(foot_positions[leg], r_min, r_max) for leg in leg_names]
    overlap = rom_regions[0].intersection(rom_regions[1]).intersection(rom_regions[2])
    return overlap


    
# General Functions

def polar_to_cartesian(r, theta):
    """
    Converts polar coordinates to cartesian coordinates
    Inputs: r (cm), theta (deg)
    Outputs: x (cm), y (cm)
    """
    theta = math.radians(theta)

    x = r * math.cos(theta)
    y = r * math.sin(theta)

    return np.array([x, y])

