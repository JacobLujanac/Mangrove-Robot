# Servo Calibration
from MotorClass_Servo2040 import Motor
import LegClass_Servo2040 as LegClass
from servo import Servo, servo2040
import config, time
from machine import Pin

legName = "RM"

legNum_dict = {
    "RB": 1, "LB": 2, "RM": 3,
    "LM": 4, "RF": 5, "LF": 6
    }

if legName[0] == "L":
        sideVal = "Left"
if legName[0] == "R":
        sideVal = "Right"
        
legNum = legNum_dict[legName]
# Turn on Relay
relay_pin = Pin(27,Pin.OUT) # A1 --> COM
relay_pin.value(1)
time.sleep(0.5)




# - Hip - #
servo = Motor((legNum-1)*3,"Hip",legName,invert = False, side = sideVal)

# - Knee - #
#servo = Motor((legNum-1)*3 + 1,"Knee",legName,invert = False, side = sideVal)

# - Ankle - #
#servo = Motor((legNum-1)*3+ 2,"Ankle",legName,invert = True, side = sideVal)


servo.servo.value(-15)

#servo.set_deg(20)

print(f"Degree: {servo.get_deg()}")
#servoA.set_deg(70)
#servoB.set_deg(-90)


# 
## -- Knee -- ##
#-Motor Test-#
# servo = Motor(0,"Knee","RB",side = "Right")    
# servo.servo.value(-3)
# #
# #-Leg Test-#
# servo = Motor(0,"Knee","RB",invert = False, side = "Right")
# servo.set_deg(-10)




# ## -- Ankle (Left) -- ##
# #-Motor Test-#
 
# servo = Motor(0,"Ankle","LM", side = "Left")    
# servo.servo.value(-45)
#
# # #-Leg Test-#
# servo = Motor(17,"Ankle","LF",invert = True, side = "Left")
# servo.set_deg(-90)




## -- Ankle (Right) -- ##
# #-Motor Test-#
# servo = Motor(0,"Ankle","RB")    
# servo.servo.value(25)

# #-Leg Test-#
# servo = Motor(0,"Ankle","RB",invert = True, side = "Right")
# servo.set_deg(-90)





# # === Helper ===
# def get_leg_by_name(name):
#     for leg in legs:
#          if leg.name == name:
#             return leg
#     raise ValueError(f"[Servo2040 Error] Leg with name '{name}' not found")
# 
# # === SETUP LEGS ===
# legs = []
# LEG_NAMES = config.LEG_NAMES
# print(LEG_NAMES)
# 
# # Assign Motors to Legs
# for i in range(len(LEG_NAMES)):
#     name = LEG_NAMES[i]
#     servo_indices = [i*3, i*3 + 1, i*3 + 2] 
#     legs.append(LegClass.Leg(name, servo_indices, side = "Right"))
#     
#     # Zero All Joints
#     leg = get_leg_by_name(name)
#     leg.set_angles(0,0,0)
#     
#     
# #     print(leg.motors["knee"].offset)
# #     #leg.motors["knee"].set_deg(90)
# #     print(leg.motors["knee"].get_deg())
# #     
#     
#     
