import sys, os, time
sys.path.append(os.path.abspath("."))
from Software.Core.Robot import *
from Software.Comms.trajectory_sender import Servo2040Interface
from Software.GaitAlgorithm.GaitAlgorithm import gait_algorithm
import numpy as np



# # Initialize Servo2040 Connection
# servo_interface = Servo2040Interface()
# servo_interface.connect()


# # Initialize Robot
# groveBot = Robot(servo_interface)
groveBot = Robot()

gait_algorithm(groveBot, visualize= "False")

# time.sleep(0.2)


# plant_coords = [
#     np.array([10,8]),
#     np.array([-10,3]),
#     np.array([10,-8])]

# groveBot.plant(plant_coords)


