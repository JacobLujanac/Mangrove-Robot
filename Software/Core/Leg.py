import sys, os
sys.path.append(os.path.abspath("."))
from Software.GeneralKinematics.kinematics import *
from Software.GeneralKinematics.trajectory import * 
from Software.Core.math_utils import *
import Software.Core.config as config
import numpy as np


class Leg:
    def __init__(self, robot,name, servo_ids):
        self.name = name
        self.servo_ids = servo_ids
        self.limb_lengths = robot.limb_lengths
        self.joint_ROM_limits = robot.joint_ROM_limits
        self.hip_offset = config.HIP_OFFSETS[name]
        self.hip_pos = config.HIP_POS[name]
        self.joint_angles = {"hip": self.hip_offset , "knee": 70, "ankle": -90} # Raise legs up to avoid colliding with table
        self.steps =  config.steps
        
        self.z_root = config.z_root
        self.z_bodyClearance = config.z_bodyClearance
        self.plant_status = None
        
    
    def update_current_angles(self, angles):

        self.joint_angles["hip"] = angles[0]
        self.joint_angles["knee"] = angles[1]
        self.joint_angles["ankle"] = angles[2]

    def update_plant_status(self,status):
        self.plant_status = "Planted" if status == True else "Unplanted"
    
    def get_plant_status(self):
        return self.plant_status
        
    def get_current_angles(self):
        
        return list(self.joint_angles.values())
    

    def get_joint_trajectory(self, traj_fn, ease_fn, start, end, space="rz", move_type = "Feet"):
        """
        Generates a joint angle trajectory from a Cartesian path.
        space: "rz" or "xy"
        """
        trajCoords = gen_trajectory(traj_fn, ease_fn, start, end, self.steps, leg = self, move_type = move_type)
               
        if space == "rz":
            theta_hip, _, _ = self.get_current_angles()
            theta_sol = ik_rz(leg=self, rz_foot=trajCoords)
            theta_knee = theta_sol[:, 0]
            theta_ankle = theta_sol[:, 1]
            theta_hip_array = np.full((len(theta_knee),), theta_hip)
            solutions = np.stack((theta_hip_array, theta_knee, theta_ankle), axis=1)

        elif space == "xy":
            _, z = fk_rz(self)
            z_array = np.full((trajCoords.shape[0],), z)
            xyzCoords = np.hstack((trajCoords, z_array[:, np.newaxis]))
            solutions = ik_xyz(leg=self, xyz_foot=xyzCoords)

        elif space == "xyz":
            solutions = ik_xyz(leg=self, xyz_foot=trajCoords)

      
        return solutions
    
# === Movement Functions === #

    def foot_lower(self):
        r, z = fk_rz(self)
        z_root = self.z_root
        z_bodyClearance = self.z_bodyClearance
        return self.get_joint_trajectory(
            lin_trajectory, ease_out_quad,
            start=[r, z],
            end=[r, -1*(z_root+z_bodyClearance)],
            space="rz"
        )

    def foot_raise(self):
        r, z = fk_rz(self)
        z_root = self.z_root
        return self.get_joint_trajectory(
            lin_trajectory, ease_in_quad,
            start=[r, z],
            end=[r, -1 * self.z_bodyClearance],
            space="rz"
        )
    
    def leg_swing(self,xy_coord,move_type, traj_fn = lin_trajectory):
        
        """
        xy_fin: Global xy coordinates to move end effector to
        move_type: whether the foot or hip is directed along the trajectory coordinates
                 "moveFoot": origin at hip
                 "moveHip": relative displacement from hip's starting position
        """

        if move_type == "moveFeet":
            xy_start = np.array(fk_xyz(self)[:2])

        elif traj_fn == lin_trajectory:
           xy_start = np.array([0,0])

        elif traj_fn == rotation_arc_trajectory:
            xy_start = self.robot.yaw

        return self.get_joint_trajectory(
            traj_fn, ease_sin,
            start= xy_start,
            end= xy_coord,
            space="xy",
            move_type = move_type)
    

   
