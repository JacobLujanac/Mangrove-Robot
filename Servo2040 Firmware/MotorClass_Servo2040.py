from servo import Servo, servo2040
import config


class Motor:
    def __init__(self, index, joint_type, leg_name, invert=False, side="Left"):
        self.joint_type = joint_type
        self.leg_name = leg_name
        self.invert = invert
        self.side = side

        self.JOINT_OFFSETS = config.JOINT_OFFSETS
        self.GLOBAL_OFFSETS = config.GLOBAL_OFFSETS
        self.ROM_LIMITS = config.ROM_LIMITS

        # Offset between global robot coordinate frame and local hip mount orientation
        self.hip_offset = self.GLOBAL_OFFSETS["Hip"][leg_name] if joint_type == "Hip" else 0

        # Mechanical mounting offset for aligning servo position with physical joint angle
        self.offset = self.JOINT_OFFSETS[joint_type][leg_name]
        

        self.rom_min, self.rom_max = self.ROM_LIMITS[joint_type]

        # Initialize and calibrate servo
        self.servo = Servo(getattr(servo2040, f"SERVO_{index + 1}"))
        cal = self.servo.calibration()
        if joint_type == "Hip" or (self.leg_name == "LB" and self.joint_type == "Ankle"):
            cal.first_value(-135)
            cal.last_value(135)
        
        else:
            cal.first_value(-90)
            cal.last_value(90)
        cal.limit_to_calibration(True, True)
        self.servo.calibration(cal)

       
        

    def set_deg(self, deg):
        # Convert world-space joint angle to servo-space command
        if self.joint_type == "Hip":
            deg -= self.hip_offset
            #print(f"{self.name}: {deg}")

        deg_adj = deg - self.offset
        #print(f"{self.name}: {deg_adj}")
        
        if self.joint_type == "Hip":
            deg_adj = -deg_adj # Hip servos are oriented to face the -z axis
        else:
            if self.invert:
                deg_adj = -deg_adj       
            if self.side == "Right":
                deg_adj = -deg_adj
        
            
        #print(f"{self.name}: {deg}")
        self.servo.value(deg_adj)

    def get_deg(self):
        # Convert servo-space reading back to world-space joint angle
        deg = self.servo.value()
        
        if self.joint_type == "Hip":
            deg = -deg
            
        else:
            if self.side == "Right":
                deg = -deg
            if self.invert:
                deg = -deg
                
        #print(f"{self.name}: {deg}")
        
        if self.joint_type == "Hip":
            deg += self.hip_offset
        
        deg_adj = deg + self.offset          
        #print(f"{self.name}: {deg_adj}")
        return deg_adj




