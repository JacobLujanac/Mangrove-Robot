# Imports
import time, math
from servo import Servo, servo2040
from pimoroni import Button



# Set up Motor class
class Motor:
    def __init__(self, index, joint ):
        
     
        """
        Initializes servos

        index: Servo index (0-17) corresponding to its position.
        """
        
        self.index = index
        self.joint = joint
      
        
    
        
        # Assign to pin
        num = index + 1
        self.pin = f"servo2040.SERVO_{num}"
        
        
        # Create Servo Object
        self.s = Servo( eval(self.pin) ) # Initialize the servo on the given index 


        # Calibration for 270-degree movement
        cal = self.s.calibration()
      
      
        cal.first_value(-135)
        cal.last_value(135)
        cal.limit_to_calibration(True,True)
        
        
        # Apply Calibration
        self.s.calibration(cal)
    
    @classmethod
    def incrementStep(cls):
        clc.step += 1;
        
    @classmethod
    def resetStepCount(cls):
        clc.step = 0
    
    @classmethod
    def setTotalSteps(cls,steps):
        clc.totalSteps = steps
    
    def __getattr__(self,attr):
        return getattr(self.s, attr)
    
    def setDeg(self,deg):
        
            
        # Degree Limits
        minLim, maxLim = joint_limits[self.joint]
        
        
        # Constrain Input Degree
        if deg > maxLim:
            deg = maxLim
         
         
        if deg < minLim:
            deg = minLim
         
         
        # Apply Degree to Servo
        self.s.value(deg)
        
    def getDeg(self):
        return self.s.value() 

# Initialize all motors


# Create Motors List
motors = []

# Populate Motors List
for i in range(servo2040.NUM_SERVOS):
    
    # Assign Joint Type
    joint = ["Hip", "Knee", "Ankle"][i % 3]
    

    
    # Add Motor to List
    motors.append( Motor(i, joint))
    
# Create Legs List
legs = []


# Assemble Legs
for i in range(6):
    
    # Cycle Through Name
    name = ["RightFront", "RightMiddle", "RightBack", "LeftFront", "LeftMiddle", "LeftBack"][i]
    
    # Add Leg to List
    legs.append( Leg(name, motors[i*3], motors[i*3 + 1], motors[i*3 + 2] ) )

print(legs)


class Robot(self, legs)

    self.RF = legs[0]
    self.RM = legs[1]
    self.RB = legs[2]
    self.LF = legs[3]
    self.LM = legs[4]
    self.LB = legs[5]
    
