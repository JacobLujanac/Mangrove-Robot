import Robot

class Leg(RoboBrain):
    def __init__(self, name, hipServo, kneeServo, ankleServo, anlgeOffsets=[0 ,0, 0]):
        
        self.name = name
        self.hipServo = hipServo
        self.kneeServo = kneeServo
        self.ankleServo = ankleServo

        self.coxLen = 5.566; #cm
        self.femLen = 8.679; #cm
        self.tibLen = 7.042; #cm
