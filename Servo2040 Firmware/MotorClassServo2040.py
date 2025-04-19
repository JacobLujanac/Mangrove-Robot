class Motor:
    def __init__(Leg, index, jointType):
        
     
        """
        Initializes servos

        index: Servo index (0-17) corresponding to its position.
        """
        
        self.index = index
        self.jointType = jointType
      
        #####################
        # Joint Properties   
        #####################

        # Convert Coord Systems

        offset = {
        "Hip": {
          "F": 90
          "RF": 45
          "R": 0
          "RB": -45
          "B": -90
          "LB": -135
          "L": -180
          "LF": 135
        },
        "Knee": , 
        "Ankle": (0,135),
        }

        # Angle Limits
        RoM = {
        "Hip": (-90,90),
        "Knee": (-90,135), 
        "Ankle": (0,135),
        }

      
      
      
    joint_limits = {
    "Hip": (-90,90),
    "Knee": (-90,135), 
    "Ankle": (0,135),
    
    }
        
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
        minLim, maxLim = jointType_limits[self.jointType]
        
        
        # Constrain Input Degree
        if deg > maxLim:
            deg = maxLim
         
         
        if deg < minLim:
            deg = minLim
         
         
        # Apply Degree to Servo
        self.s.value(deg)
        
    def getDeg(self):
        return self.s.value() 
