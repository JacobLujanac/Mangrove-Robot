from motor import Motor

class Leg:
    def __init__(self, name, servo_indices):
        self.name = name
        self.motors = {
            "hip": Motor(servo_indices[0], "Hip", name),
            "knee": Motor(servo_indices[1], "Knee", name),
            "ankle": Motor(servo_indices[2], "Ankle", name)
        }

    def set_angles(self, hip, knee, ankle):
        self.motors["hip"].set_deg(hip)
        self.motors["knee"].set_deg(knee)
        self.motors["ankle"].set_deg(-ankle) # Account for how the servo itself moves, while the horn stays fixed)



