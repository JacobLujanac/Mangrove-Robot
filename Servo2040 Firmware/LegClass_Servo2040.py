from MotorClass_Servo2040 import Motor
from machine import Pin
import config

class Leg:
    def __init__(self, name, servo_indices, side, touch_sensor_pin):
        self.side = side
        self.name = name
        self.motors = {
            "hip": Motor(servo_indices[0], "Hip", name, invert = False, side = side),
            "knee": Motor(servo_indices[1], "Knee", name, invert = False, side = side),
            "ankle": Motor(servo_indices[2], "Ankle", name, invert = True, side = side) # Account for how the servo itself moves, while the horn stays fixed)
        }
        
        self.touch_sensor = Pin(touch_sensor_pin, Pin.IN, Pin.PULL_UP)

    def set_angles(self, hip, knee, ankle):
        
        self.motors["hip"].set_deg(hip)
        self.motors["knee"].set_deg(knee)
        self.motors["ankle"].set_deg(ankle) 
        
    def get_angles(self):
        
        hipAngle = self.motors["hip"].get_deg()
        kneeAngle = self.motors["knee"].get_deg()
        ankleAngle = self.motors["ankle"].get_deg()
        
        print(f"Hip Angle:{hipAngle}")
        print(f"Knee Angle:{kneeAngle}")
        print(f"Anklel Angle:{ankleAngle}")
        return hipAngle, kneeAngle, ankleAngle
    
    def read_touch_sensor(self):
        """
        Output = 
            True: Touch sensor is pressed
            False: Touch sensor is not unpressed
        """
        return self.touch_sensor.value() == 0 

        





