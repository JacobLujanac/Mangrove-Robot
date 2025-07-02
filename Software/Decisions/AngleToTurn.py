import math
import numpy as np


# REVISIONS TO MAKE
    # robo_loc -> get from accelerometer
    # robo_ang -> get from accelerometer
    # targ_loc -> get from path planning script
    # targ_tol -> get from path planning script


###############################################################
#               Variable Initialization                      #
###############################################################

# UPDATE (get orientation and position from accelerometer)

robo_loc = np.array([0,0]) # GET FROM ACCELEROMETER
robo_ang = np.array([-1,5]) # GET FROM ACCELEROMETER

targ_loc = np.array([0,5]) # GET FROM PATH PLANNING SCRIPT
targ_tolerance = 0.5 # GET FROM PATH PLANNING SCRIPT

# Normalize into unit vectors
def unit_vec(vec):
    u = vec / math.hypot(*vec)
    return u


robo_dir = unit_vec(robo_ang)
vec_targ = unit_vec(targ_loc - robo_loc)   


vec_targ_perp = np.array([-vec_targ[1],vec_targ[0]]) # Perpendicular direction, use to establish tolerance range

lim1 = targ_loc + (targ_tolerance * vec_targ_perp)
lim2 = targ_loc - (targ_tolerance * vec_targ_perp)

vec_lim1 = unit_vec(lim1 - robo_loc)  
vec_lim2 = unit_vec(lim2 - robo_loc)  

# Calculate angle between vectors


def angleBetweenVec(vec1,vec2):
    
    '''
    Calculates the angle between two unit vectors

    A cross B = |A||B|sin(theta)
    --> theta = arcsin(A cross B)
    '''

    theta = math.asin(np.cross(vec1 ,vec2))
    theta = math.degrees(theta)

    return theta

theta_targetRange = abs( angleBetweenVec(vec_lim1,vec_lim2) )
theta_roboToTarg = angleBetweenVec(robo_dir,vec_targ)

###############################################################
#               Movement Decision                             #
###############################################################

if (abs(theta_roboToTarg) <= (theta_targetRange/2)):
    Movement = "Forward"

    print(f"Movement ={Movement}")

else:
    Movement = "Rotate"
    theta_rotate = theta_roboToTarg

    print(f"Movement ={Movement}")
    print(f"/theta_rotate = {theta_rotate}")
    print(f"/theta_range = {theta_targetRange/2}")    

