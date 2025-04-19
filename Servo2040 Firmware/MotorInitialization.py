# Imports
import time, math
from servo import Servo, servo2040
from pimoroni import Button




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
    
