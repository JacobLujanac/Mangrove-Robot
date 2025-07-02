from servo import Servo, servo2040
import config


class Motor:
    def __init__(self, index, joint_type, leg_name, invert = False, side = "Left"):
        self.joint_type = joint_type
        self.leg_name = leg_name
        self.invert = invert
        self.side = side
        
        self.JOINT_OFFSETS = config.JOINT_OFFSETS
        self.ROM_LIMITS = config.ROM_LIMITS
        

        self.offset = (
            self.JOINT_OFFSETS[joint_type][leg_name]
            if joint_type == "Hip" else self.JOINT_OFFSETS[joint_type]
        )
        self.rom_min, self.rom_max = self.ROM_LIMITS[joint_type]

        # Initialize motors
        self.servo = Servo(getattr(servo2040, f"SERVO_{index + 1}"))

        # Setup calibration
        # NEED TO INCLUDE 180 AND 270
        cal = self.servo.calibration()
        cal.first_value(-135)
        cal.last_value(135)
        cal.limit_to_calibration(True, True)
        self.servo.calibration(cal)
        self.set_deg(0)

    def set_deg(self, deg):
        
        
            
        
            
       
        
        
        #Convert from Worldspace coordinates to Motor cordinates
        if self.invert:
            deg = -deg
        if (self.side == "Right") and (self.joint_type != "Hip"):
            deg = -deg
            
        deg_adj = deg + self.offset
        self.servo.value(deg_adj)
        
        
    def get_deg(self):
            
        deg_adj = self.servo.value() - self.offset
        
        
        if self.invert:
            deg_adj = -deg_adj
        if (self.side == "Right") and (self.joint_type != "Hip"): #Degree inversion is to compensate for mirrored rz plane (hip operates on xy plane, so not effected)
            deg_adj = -deg_adj
            
        return deg_adj



