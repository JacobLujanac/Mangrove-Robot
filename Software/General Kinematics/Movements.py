import numpy as np
import math

###############################
## PUT THESE IN ROBO CLASS (so can easily references its own properties, like limb length)
#########################
#########################
#########################
#########################
#########################

####################
# moved from body to brain


# Joint ROM Limits
# joint_limits = {
#     "Hip": (-90,90),
#     "Knee": (-90,135), #This is with 0 being horizontal (parallel to ground)
#     "Ankle": (0,135),
    
    # }

class DegMath:
    @staticmethod
    def sin(deg):
        return np.sin(np.radians(deg))

    @staticmethod
    def cos(deg):
        return np.cos(np.radians(deg))

    @staticmethod
    def tan(deg):
        return np.tan(np.radians(deg))

    @staticmethod
    def arcsin(x):
        return np.degrees(np.arcsin(x))

    @staticmethod
    def arccos(x):
        return np.degrees(np.arccos(x))

    @staticmethod
    def arctan2(y, x):
        return np.degrees(np.arctan2(y, x))
# EASING FUNCTIONS

# Cubic Smoothstep
def ease_in_out(t):
    return 3 * t**2 - 2 * t**3

# Quadratic Out
def ease_out_quad(t):
    return 1 - (1 - t)**2

# Quadratic In
def ease_in_quad(t):
    return t**2

# Sinusoidal
def ease_sin(t):
    return 0.5 * (1 - np.cos(np.pi * t))






#############################################################################################################################################
# TRAJECTORY GENERATION
##############################################################################


# Trajectory Generator
def gen_trajectory(traj_fn,ease_fn,start_pos,end_pos,steps,footPlanted=False):

    if footPlanted == True:
        end_pos = -end_pos # Hip moves relative to foot
    
    # Generate Trajectory
    traj = traj_fn(start_pos,end_pos,steps)

   # Apply easing
    eased_coords = []

    for i in range(steps):
        t = i / (steps - 1)
        eased_t = ease_fn(t)
        idx = eased_t * (steps - 1)
        low = int(np.floor(idx))
        high = min(low + 1, steps - 1)
        alpha = idx - low

        interp = (1 - alpha) * traj[low] + alpha * traj[high]
        eased_coords.append(interp)

    return np.array(eased_coords)

# TRAJECTORY TYPES

def lin_trajectory(start_pos, end_pos, steps):

    """Generate coordinates for a straight line from start_pos to end_pos"""


    coord1 = np.linspace(start_pos[0], end_pos[0], steps)
    coord2 = np.linspace(start_pos[1], end_pos[1], steps)

    trajectory = np.stack((coord1, coord2), axis=-1)

    return trajectory 



# Practical IK Solution Filtering


def closest_solution(all_solutions, initialAngles):
    """
    Selects the smoothest sequence of IK solutions across a trajectory.
    
    Parameters:
        all_solutions (ndarray): shape (n_steps, n_options, n_joints)
            The full set of IK solutions at each time step.
        joint_index (int): index of joint to compare for "closeness" (default: 0)

    Returns:
        ndarray: shape (n_steps, n_joints), the filtered smooth trajectory.
    """
    n_steps, n_options, n_joints = all_solutions.shape
    chosen = []

    options = all_solutions[0]
    distances = np.abs(options[:, joint_index] - initialAngles[joint_index])
    best_idx = np.argmin(distances)
    selected = options[best_idx]

    chosen.append(selected)

    for i in range(1, n_steps):
        prev = chosen[-1]
        options = all_solutions[i]

        
        distances = np.linalg.norm(options - prev, axis=1)
        best_idx = np.argmin(distances)
        chosen.append(options[best_idx])

    return np.array(chosen)  # shape (n_steps, n_joints)


#############################################
# Generic Kinematic Functions
#############################################


def fk_xy(self, leg):

    l_leg, _ = fk_rz(leg)


    """
    Returns the location of the foot on the xy-plane 

    Coordinate Frame: Local (Origin at q_robot)

    Parameters
    """

    # Location of hip
    [x_hip, y_hip] = leg.q_hip

    # Angle of coxa
    theta_hip = leg.theta_hip

    # Location of foot
    x = x_hip + l_leg * DegMath.cos(theta_hip)
    y = y_hip + l_leg * DegMath.sin(theta_hip)
    return x, y


def ik_xyz(self, xyz_foot, leg):
    """
    Coordinate Frame: Local (Origin at q_robot)

    Parameters:
        xyz_foot (array-like of float): [x, y, z] position of foot

    Returns:
        (theta_hip, theta_knee, theta_ankle): 
            theta_hip (float): angle of the femur from the x-axis (degrees)
            theta_knee (float): angle of the femur from the r-axis (degrees)
            theta_ankle (float): internal angle between femur and tibia (degrees)
    """
    x = xyz_foot[:,0]
    y = xyz_foot[:,1]
    z = xyz_foot[:,2]

    x_hip = leg.q_hip[0]
    y_hip = leg.q_hip[1]

    # Angle of hip needed
    theta_hip = DegMath.arctan2( (y-y_hip) / (x-x_hip))

    # Radial length of leg needed
    r = np.hypot(x,y)

    # Knee and ankle angles
    theta_knee, theta_ankle = ik_rz(r,z)

    return np.stack((theta_hip,theta_knee,theta_ankle), axis = 1)




#############################################
# rz plane movement
#############################################

def fk_rz(self,leg):

    """

    Coordinate System: Local (Origin at q_hip)

    Forward kinematics for a 2R planar manipulator

    
    Parameters: 
        theta_knee (float): angle of the femur from the r-axis (degrees)
        theta_ankle (float): internal angle between femur and tibia (degrees)

    Returns:
        r: radial position of foot
        z: vertical position of foot

    """
    theta_knee = leg.theta_knee
    theta_ankle = leg.theta_ankle

    l_femur = self.l_femur
    l_tibia = self.l_tibia

    # Compute radial position of foot
    r = ( l_femur * DegMath.cos(theta_knee) ) + ( l_tibia * DegMath.cos(theta_knee + theta_ankle) )

    # Compute vertical position of foot
    z = ( l_femur * DegMath.sin(theta_knee) ) + ( l_tibia * DegMath.sin(theta_knee + theta_ankle) )

    return r, z



def ik_rz(self, rz_foot):
    """
    Inverse kinematics for a 2R planar manipulator

    Coordinate Frame: Local

    Parameters:
        r (float): Target r position of foot
        z (float): Target z position of foot

    Returns:
        (theta_knee, theta_ankle): 
            theta_knee (float): angle of the femur from the r-axis (degrees)
            theta_ankle (float): internal angle between femur and tibia (degrees)
    """
    r = rz_foot[:,0]
    z = rz_foot[:,1]

    l_femur = self.l_femur
    l_tibia = self.l_tibia

    # Compute distance to the target point
    d = np.sqrt(r**2 + z**2)
    D = (d**2 - l_femur**2 - l_tibia**2) / (2 * l_femur * l_tibia)

    # Check if the point is reachable
    if abs(D) > 1.0:
        raise ValueError("Target point is out of reach.")

    # Compute theta_ankle  (both ankle up and ankle down solutions)
    theta_ankle_options = [DegMath.arccos(D), -DegMath.arrcos(D)]
    

    # Compute theta_knee
    for theta_ankle in theta_ankle_options:

        k1 = l_femur + l_tibia * DegMath.cos(theta_ankle)
        k2 = l_tibia * DegMath.sin(theta_ankle)
        theta_knee_options = DegMath.arctan2(z, r) - DegMath.arctan2(k2, k1)

    
    # Seperate by solution
    
    solutions = np.stack((theta_knee_options, theta_ankle_options[:,1]),axis = 2)

    initialAngles = [leg.knee.getDeg, leg.ankle.getDeg]
    
    jointAngles = closest_solution(solutions,initialAngles)

    return jointAngles



#########################################################################
# Overall Movement Command
##################################################################

# Will include the automatic angle clamping
# FIX BELOW (don't send to servo quite yet, this function is just to enforce ROM limits)
def clamp_angle(angle, joint_index, leg_obj):
    min_angle, max_angle = leg_obj.joint_rom[joint_index]
    return max(min(angle, max_angle), min_angle)

# Apply clamp per joint before sending
for leg_index, angles in enumerate(joint_angles):
    leg = leg_objs[leg_index]
    clamped_angles = [
        clamp_angle(angles[i], i, leg) for i in range(3)
    ]
    send_to_servo2040(leg.motor_ids, clamped_angles)
######################
# Movement Presets
#####################

def footLower(leg):
    
    """
    Lowers leg in straight line to ground (z=0)
    """

    # Current rz position of foot
    r, z = fk_rz(leg)

    # Inputs for trajectory generation
    rzStart = [r,z]
    rzEnd = [r,0]    

    steps = leg.steps

    # Generate trajectory
    traj = gen_trajectory(lin_trajectory,ease_out_quad,rzStart,rzEnd, steps)

    # Solve for joint angles
    jointAngles = ik_rz(traj)

    return jointAngles

def footRaise(self,leg):
    
    """
    Raises leg in straight line to vertically clear the roots
    """

    # Current rz position of foot
    r, z = fk_rz(leg)

    # Height to clear roots
    z_roots = self.z_roots

    # Inputs for trajectory generation
    rzStart = [r,z]
    rzEnd = [r,z_roots]    

    steps = leg.steps

    # Generate trajectory
    traj = gen_trajectory(lin_trajectory,ease_in_quad,rzStart,rzEnd, steps)

    # Solve for joint angles
    jointAngles = ik_rz(traj)


    return jointAngles


def hipSwingPlanted(leg,endPos):

    """
    While foot is planted, swings hip in xy plane
    """
    
    #
    # Inputs for trajectory generation
    xyStart = fk_xy(leg) # Current location of foot
    xyEnd = endPos # Foot origin is already accounted for in gen_trajectory
  
    steps = leg.steps

    # Generate trajectory
    traj = gen_trajectory(lin_trajectory,ease_out_quad,rzStart,rzEnd, steps)


    jointAngles = ik_xyz(endPos)
    
   
    # Solve for joint angles
    jointAngles = ik_rz(traj)

    # Filter solution
    jointAngles = closest_solution(jointAngles)

    return jointAngles








######################################
# Execution Structure
#################################

# coord_traj = gen_trajectory(start_coord, end_coord, steps, easing_fn)
# angle_traj = [ik_solver(coord) for coord in coord_traj]

## TESTING
#################################

