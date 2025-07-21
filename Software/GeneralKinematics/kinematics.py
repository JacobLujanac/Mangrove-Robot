# Kinematics for general appendage
# Origin: hip
import sys
import os
sys.path.append(os.path.abspath("."))
import numpy as np
from Software.Core.math_utils import DegMath


# rz plane FK/IK

def fk_rz(leg):
    
    theta_knee = leg.joint_angles["knee"]
    theta_ankle = leg.joint_angles["ankle"]
    
        
    l_femur = leg.limb_parameters["l_femur"]
    l_tibia = leg.limb_parameters["l_tibia_eff"]
    
    r = (l_femur * DegMath.cos(theta_knee) + l_tibia * DegMath.cos(theta_knee + theta_ankle))
    z = (l_femur * DegMath.sin(theta_knee) + l_tibia * DegMath.sin(theta_knee + theta_ankle))


    return r, z



def ik_rz(leg, rz_foot):
    
    r = rz_foot[:, 0]  
    z = rz_foot[:, 1]
    
        
    l_femur = leg.limb_parameters["l_femur"]
    l_tibia = leg.limb_parameters["l_tibia_eff"]
    
    d = np.sqrt(r**2 + z**2) 
    D = (d**2 - l_femur**2 - l_tibia**2) / (2 * l_femur * l_tibia) 
 
    if np.any(np.abs(D) > 1):
        raise ValueError("One or more target points are out of reach.")

    theta_ankle = -DegMath.arccos(D)
   
    k1 = l_femur + l_tibia * DegMath.cos(theta_ankle)  
    k2 = l_tibia * DegMath.sin(theta_ankle)
    
    theta_knee = DegMath.arctan2(z, r) - DegMath.arctan2(k2, k1) 
  
    solution = np.stack([theta_knee, theta_ankle], axis=1)  

    return solution 




#xyz Kinematics

def fk_xyz(leg):
    
    l_leg, z = fk_rz(leg)
    theta_hip = leg.joint_angles["hip"]
    
    x = l_leg * DegMath.cos(theta_hip)
    y = l_leg * DegMath.sin(theta_hip)
    
    
    return x, y, z

# xyz IK

def ik_xyz(leg, xyz_foot):
    
    x = xyz_foot[:, 0]
    y = xyz_foot[:, 1]
    z = xyz_foot[:, 2]

    theta_hip = DegMath.arctan2(y,x)
    
    r = np.hypot(x, y)
   
    sol = ik_rz(leg, np.column_stack((r, z)))
    theta_knee = sol[:,0]
    theta_ankle = sol[:,1]
 
 
    return np.stack((theta_hip, theta_knee, theta_ankle), axis=1)


### angle_clamp.py
def clamp_angle(angle, joint_index, leg_obj):
    min_angle, max_angle = leg_obj.joint_rom[joint_index]
    return max(min(angle, max_angle), min_angle)
