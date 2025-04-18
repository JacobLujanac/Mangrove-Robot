# Imports
import time, math
from servo import Servo, servo2040
from pimoroni import Button


#for i in range(18):


# Joint ROM Limits
joint_limits = {
    "Hip": (-90,90),
    "Knee": (-90,135), #This is with 0 being horizontal (parallel to ground)
    "Ankle": (0,135),
    
    }


    
 
        

# Example usage:

motor = Motor(0,"Hip","Right")  # Create a motor object for servo 0



class Leg():
    def __init__(self, name, hipServo, kneeServo, ankleServo):
        self.name = name
        self.hipServo = hipServo
        self.kneeServo = kneeServo
        self.ankleServo = ankleServo

        
# Set Joint Angles
    def hip(self, deg):
        self.hipServo.setDeg(deg)
        self.hipServo.getDeg(deg)


    def knee(self, deg):
        self.kneeServo.setDeg(deg - 90)
        self.kneeServo.getDeg(deg - 90)

    def ankle(self, deg):
        self.ankleServo.setDeg(-deg) # Account for how the servo itself moves, while the horn stays fixed)


    


#Testing
coxaMotor = Motor(0,"Knee","Right")

coxaMotor.setDeg(135)
print(coxaMotor.pulse())
            
# Leg Creation

# # Create Motors List
# motors = []
# 
# # Populate Motors List
# for i in range(servo2040.NUM_SERVOS):
#     
#     # Assign Joint Type
#     joint = ["Hip", "Knee", "Ankle"][i % 3]
#     

#     
#     # Add Motor to List
#     motors.append( Motor(i, joint))
#     
# # Create Legs List
# legs = []
# 
# 
# # Assemble Legs
# for i in range(6):
#     
#     # Cycle Through Name
#     name = ["RightFront", "RightMiddle", "RightBack", "LeftFront", "LeftMiddle", "LeftBack"][i]
#     
#     # Add Leg to List
#     legs.append( Leg(name, motors[i*3], motors[i*3 + 1], motors[i*3 + 2] ) )
# 
# print(legs)
# 
# 
# class Robot(self, legs)
# 
#     self.RF = legs[0]
#     self.RM = legs[1]
#     self.RB = legs[2]
#     self.LF = legs[3]
#     self.LM = legs[4]
#     self.LB = legs[5]
#     
    
