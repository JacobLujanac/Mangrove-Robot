import sys, os, math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
from Software.Core.math_utils import *
import numpy as np
from shapely.geometry import Point
from shapely.ops import unary_union
from shapely.geometry import Polygon
from shapely.affinity import translate
from functools import reduce
import heapq
from collections import deque



# Robot specific
def get_rotation_matrix(yaw):
    return np.array([
        [DegMath.cos(yaw), -DegMath.sin(yaw)],
        [DegMath.sin(yaw),  DegMath.cos(yaw)]
    ])

def get_hip_positions(pose, tripod_legs, mode = "Global"):
    """Calculate positions of each hip based on robot pose."""

    R = get_rotation_matrix(pose["yaw"])
    center = np.array([pose["x"], pose["y"]])
    hips_pos = {}

    for leg_name in tripod_legs:
        rel_vec = R @ np.array(cfg.HIP_POS[leg_name])
        if mode == "Global": 
            rel_vec += center
        hips_pos[leg_name] = tuple(rel_vec)
    return hips_pos

def get_feet_world_positions(feet_rel_to_cog, hip_positions, robot_pose):
    R = get_rotation_matrix(robot_pose["yaw"])
    feet_world = {}
    CoG = np.array([robot_pose["x"],robot_pose["y"]])
    for leg, foot_local in feet_rel_to_cog.items():
        
        foot_offset = np.array(foot_local)
        foot_world = foot_offset + CoG 
        feet_world[leg] = tuple(foot_world)
    return feet_world
   
# General
def is_valid(viable_zone,x, y, psi):
    return Point(x, y).within(viable_zone[psi])



def greedy_bfs(start,goal_direction, viable_zones, grid_resolution=(0.5, 0.5, 5)):
    

    dx, dy, dpsi = grid_resolution
    visited = set()
    came_from = {}
    
    def heuristic(node):
        # Use dot product to prioritize movement in the goal direction
        direction = np.array([node[0] - start[0], node[1] - start[1], node[2] - start[2]])
        return -np.dot(direction, goal_direction)  # Negate to make heapq a min-heap

    heap = []
    heapq.heappush(heap, (heuristic(start), start))
    visited.add(start)

    best_node = start
    best_score = -np.inf
    
    while heap:
        _, current = heapq.heappop(heap)
        
        # Measure how far along goal_direction this node is from the start
        direction_vector = np.array(current) - np.array(start)
        projection = np.dot(direction_vector, goal_direction)

        if projection > best_score:
            best_score = projection
            best_node = current



        for dx_i in [-dx, 0, dx]:
            for dy_i in [-dy, 0, dy]:
                for dpsi_i in [-dpsi, 0, dpsi]:
                    
                    if dx_i == dy_i == dpsi_i == 0:
                        continue

                    neighbor = (
                        round(current[0] + dx_i, 3),
                        round(current[1] + dy_i, 3),
                        round(current[2] + dpsi_i, 3)
                    )

                    #print("Checking neighbor:", neighbor)
                    #print("Viable yaw?", neighbor[2] in viable_zones)
                    #if neighbor[2] in viable_zones:
                        #print("Point in polygon?", Point(neighbor[0], neighbor[1]).within(viable_zones[neighbor[2]]))

                    if neighbor in visited:
                        continue
                    if neighbor[2] not in viable_zones:
                        continue
                    if not Point(neighbor[0], neighbor[1]).within(viable_zones[neighbor[2]]):
                        continue

                    visited.add(neighbor)
                    came_from[neighbor] = current
                    heapq.heappush(heap, (heuristic(neighbor), neighbor))

    # Reconstruct path from best node
    path = [best_node]
    while best_node in came_from:
        best_node = came_from[best_node]
        path.append(best_node)
    path.reverse()
    return np.array(path)

def l_subRoot_ROM_conversion(l_subRoot):
    """
    Input: l_subRoot
    Output: r (radial length of leg required to yield l_subRoot)
    """

    r_min = cfg.r_min
    r_max = cfg.r_max

    # = Leg on Same Side of Root = #
    # Reference Points 
    point1 = [r_min, (cfg.l_subRoot_r_min[0] - cfg.l_subRoot_r_min[1])] # [r, l_subRoot] (cm)
    point2 = [r_max, (cfg.l_subRoot_r_max[0] - cfg.l_subRoot_r_max[1])] # [r, l_subRoot] (cm)

    # Slope of line
    m = (point2[1] - point1[1]) / ((point2[0] - point1[0]))

    r = ( (l_subRoot - point1[1]) / m ) + point1[0]

    
    # = Leg on Opposite Side of Root = #
    point1_op = [-point2[0],-point2[1]]
    point2_op = [-point1[0],-point1[1]]

    # Slope of line
    m_op = (point2_op[1] - point1_op[1]) / ((point2_op[0] - point1_op[0]))

    r_op = ( (l_subRoot - point1_op[1]) / m_op ) + point1_op[0]

    r_array = []
    if (r >= r_min) and (r<=r_max):
         r_array.append(r)
    if (r_op >= -r_max) and (r_op<=-r_min):
         r_array.append(r_op)

    return r_array
  

def gen_ROM_notch_shapes(center,xy_root,num_points = 50):
    """
    Parameters:
    xy_root: x and y positions of root (relative to origin: hip)
    center: foot location
    """
    x0, y0 = center
    xr, yr = xy_root

    delta_y = yr - y0
    delta_x = xr - x0

    # Root Radius [cm]
    r_r = cfg.d_r / 2

    # Distance between root and foot
    d = np.hypot(delta_x,delta_y)

    # Angle between root and foot (from positive x axis)
    theta_root = DegMath.arctan2(delta_y,delta_x)

    # Angle of Arc
    alpha = 2 * DegMath.arctan(r_r / (d - r_r))
    
    # Radius from Foot to Root 
    l_subRoot = d - r_r

    # Min value of r where collision would occur
    r_collide = l_subRoot_ROM_conversion(l_subRoot)

    arc_angle_start = theta_root - (alpha / 2)
    arc_angle_end = theta_root + (alpha / 2)

    arc_angles = np.linspace(arc_angle_start,arc_angle_end,num_points)

    notches = []
    for r in r_collide:
         
         arc1 = [(x0 + r * DegMath.cos(a), y0 + r * DegMath.sin(a)) for a in arc_angles]   
         
         if r < 0:
            points = []
            points = [(x0, y0)] + arc1 + [(x0, y0)]
            notches.append(Polygon(points))
         if r > 0:
            points = []
            arc2 = [(x0 + cfg.r_max * DegMath.cos(a), y0 + cfg.r_max * DegMath.sin(a)) for a in np.flip(arc_angles)]   
            points = arc1 + arc2
            notches.append(Polygon(points))
    return notches
def get_roots_in_bounds(center_pos, root_points, r_min = cfg.r_min, r_max = cfg.r_max):
    """Returns a list of root coordinates within ROM Bounds."""
    roots_inside = []
    for root in root_points:
        dist = np.linalg.norm(np.array(root) - np.array(center_pos))
        if r_min <= dist <= r_max:
            roots_inside.append(root)
    return roots_inside


def get_rom_region(center_pos, collidable_roots,type = "plant",r_min = cfg.r_min, r_max = cfg.r_max, resolution=64, center = "off"):
    """
    Returns a Shapely polygon representing ROM bounds
    """
    center_point = Point(center_pos)
    outer = center_point.buffer(r_max, resolution=resolution)
    inner = center_point.buffer(r_min, resolution=resolution)

    ROM = outer.difference(inner)

    if type == "translate":
    

        all_notches = []

        for root in collidable_roots:
            notches = gen_ROM_notch_shapes(center_pos,root)
            all_notches.extend(notches)

        ROM_gaps = unary_union(all_notches)

        ROM = unary_union(ROM.difference(ROM_gaps))

    # # Center ROM (so can be used in variable locations)
    # if center == "on":
    #     ROM = translate(ROM, xoff=-center_pos[0], yoff=-center_pos[1])
    
    return ROM 

def get_bos(feet_CoG_dict):
    # Get triangle vertices
    points = [feet_CoG_dict[leg] for leg in feet_CoG_dict.keys()]
    points.append(points[0])  

    BoS = Polygon(points)
    return BoS
def compute_viable_translation_zone(rom_regions):
    """
    Computes the overlap of ROMs (viable body translation region)
    from the hips of the 3 tripod legs.
    Inputs:
        foot_hip_pos: Position of foot relative to respective hip
    """
    
    overlap = reduce(lambda a, b: a.intersection(b), rom_regions)

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

