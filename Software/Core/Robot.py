from Software.Core.Leg import *
import Software.Core.config as config
from Software.Comms.trajectory_sender import *
from Software.GeneralKinematics.trajectory import lin_trajectory, ease_in_quad
import numpy as np
from functools import partial

class Robot:
    def __init__(self):
        self.servo_interface = servo_interface
        self.z_root = config.z_root
        self.z_bodyClearance = config.z_bodyClearance
        self.body_height = config.body_height
        self.legs = {}
        self.joint_ROM_limits = config.JOINT_ROM_LIMITS
        self.x_global = 15
        self.y_global = 15
        self.yaw = 50
        leg_configs = {
            "RB": {"hip": "L1_hip", "knee": "L1_knee", "ankle": "L1_ankle"},
            "LB": {"hip": "L2_hip", "knee": "L2_knee", "ankle": "L2_ankle"},
            "RM": {"hip": "L3_hip", "knee": "L3_knee", "ankle": "L3_ankle"},
            "LM": {"hip": "L4_hip", "knee": "L4_knee", "ankle": "L4_ankle"},
            "RF": {"hip": "L5_hip", "knee": "L5_knee", "ankle": "L5_ankle"},
            "LF": {"hip": "L6_hip", "knee": "L6_knee", "ankle": "L6_ankle"}
        }
        
        self.tripod_legs = config.TRIPOD_LEGS
        self.active_tripod = "A"
        
        for name, servos in leg_configs.items():
            self.legs[name] = Leg(robot=self, name=name, servo_ids=servos)

        self.defaultStance = [(self.legs["RF"].limb_parameters["l_femur"] + self.legs["RF"].limb_parameters["l_tibia_eff"]) * 0.5, -1 * (self.z_root + self.z_bodyClearance)]

        
        #self.standUp()
        print('Standing Up')

    def get_leg(self, name):
        return self.legs[name]

    def get_payload(self):
        combined = {}
        for leg in self.legs.values():
            combined.update(leg.get_servo_payload())
        return combined

    def move_legs(self, leg_movement_map, duration):
        joint_trajs = []
        legs_to_move = []

        for leg_name, movement_fn in leg_movement_map.items():
            print(f"Leg Name: {leg_name}")
            print(f"Movement Function: {movement_fn}")
            leg = self.get_leg(leg_name)
            traj = movement_fn(leg)
            joint_trajs.append(traj)
            legs_to_move.append(leg_name)

        finalData = self.servo_interface.send_joint_trajectories(legsMoving=legs_to_move, joint_trajs=joint_trajs, duration=duration)
        for message in finalData:
            if message.get("type") == "final_angles":
                # Update joint angles
                for leg_info in message["data"]:
                    leg_name = leg_info["leg"]
                    angles = leg_info["angles"]
                    if leg_name in self.legs:
                        self.legs[leg_name].update_current_angles(angles)

                # Update plant status
                touch_data = message.get("touch_data", {})
                for leg_name, status in touch_data.items():
                    if leg_name in self.legs:
                        self.legs[leg_name].update_plant_status(status)
    
        return
    

    def tripod_switch(self):
        self.active_tripod = "B" if self.active_tripod == "A" else "A"
        return
    
    
## -- Movement Functions -- ##

# - Stand Up - #
    def standUp(self):
        
        leg_names = list(self.legs.keys())
        
        def standing_plant_fn(leg):
            
            start_pos = fk_rz(leg)
            end_pos = [self.defaultStance[0], -self.body_height]
            
            return leg.get_joint_trajectory(
                lin_trajectory, ease_out_quad,
                start=start_pos,
                end=end_pos,
                space="rz"
            )    
        plant_map = {leg_name: standing_plant_fn for leg_name in leg_names}
        self.move_legs(plant_map, 1)
       
        def lift_body_fn(leg):
            r, z = fk_rz(leg)
            return leg.get_joint_trajectory(
                lin_trajectory, ease_in_quad,
                start=[r, z],
                end=self.defaultStance,
                space="rz"
            )
        lift_map = {leg_name: lift_body_fn for leg_name in leg_names}
        self.move_legs(lift_map, 3)

# - Sit Down - #
    def sitDown(self):
        
        leg_names = list(self.legs.keys())
        
       
        def lower_body_fn(leg):
          
            start_pos = fk_rz(leg)
            end_pos = [self.defaultStance[0], -self.body_height]
            return leg.get_joint_trajectory(
                lin_trajectory, ease_in_quad,
                start= start_pos,
                end= end_pos,
                space="rz"
            )
        lower_map = {leg_name: lower_body_fn for leg_name in leg_names}
        self.move_legs(lower_map, 3)

# - Plant - #
    def plant(self, plant_coords):
        
        tripod_legs = self.tripod_legs[self.active_tripod]
        """
        plant_coords: Global [x,y] Coordinates (Origin at Robot CoG)
        """
        plant_dict = {leg_name: coord for leg_name, coord in zip(tripod_legs, plant_coords)}

              
        # Repeat raising operation (if any leg is not already above root level)
        for leg_name in plant_dict:
            _,z_pos = fk_rz(self.legs[leg_name])
            
            if z_pos <= -self.z_bodyClearance:
                
                raise_map = {leg_name: Leg.foot_raise for leg_name in plant_dict}
                self.move_legs(raise_map, 0.3)
                break
        
        # Move to planting locations     
        swing_map = {
            leg_name: (lambda leg_obj = self.legs[leg_name], coords = plant_coords:
                lambda leg: leg_obj.leg_swing( (coords - np.array(config.HIP_POS[leg_obj.name])), "moveFoot")
                       )()
            for leg_name, plant_coords in plant_dict.items()
            }
     
        self.move_legs(swing_map, 0.3)
        
        # Lower Legs
        lower_map = {leg_name: Leg.foot_lower for leg_name in plant_dict}
        self.move_legs(lower_map, 0.75)
        
        time.sleep(0.5) #Allow time to ensure leg data is updated

        # Don't proceed unless all 3 legs are planted
        for leg_name in tripod_legs:
           if self.legs[leg_name] == "Unplanted":
               return
        

        # Raise legs of previously planted tripod 
        self.tripod_switch()
        prev_tripod = self.tripod_legs[self.active_tripod]
        raise_map = {leg_name: Leg.foot_raise for leg_name in prev_tripod}
        self.move_legs(raise_map, 0.3)
                
        
        
# - Translate - #
def translate(self, path):
    """
    path: xy cooredinates for CoG to translate along
    """
    tripod_legs = self.tripod_legs[self.active_tripod]

    swing_map = {
            leg_name: (lambda leg_obj = self.legs[leg_name]:
                lambda leg: leg_obj.leg_swing(path, "moveHip")
                       )()
            for leg_name in tripod_legs
            }
    self.move_legs(swing_map,0.8)
 
# - Rotate - #
def rotate(self, yaw):

    tripod_legs = self.tripod_legs[self.active_tripod]

    swing_map = {
            leg_name: (lambda leg_obj = self.legs[leg_name]:
                lambda leg: leg_obj.leg_swing(yaw, "moveHip", traj_fn = rotation_arc_trajectory)
                       )()
            for leg_name in tripod_legs
            }
    self.move_legs(swing_map, 0.8)

    # Update yaw        
    self.yaw += yaw
    
    

