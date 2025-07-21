  # === Pretabulate l_subRoot === #
import numpy as np
import sys, os, json
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg
import Software.GeneralKinematics.kinematics as kin
        
r_range = np.linspace(cfg.r_min, cfg.r_max, 100) # All possible r values      
z_planted = np.full(r_range.shape,-1*(cfg.z_root+cfg.z_bodyClearance)) # z value of planted foot    
rz_array = np.stack(([r_range,z_planted]), axis = 1)




joint_thetas = kin.ik_rz(list(robot.legs.values())[0],rz_array)
theta_knee = joint_thetas[:,0]
theta_ankle = joint_thetas[:,1]

l_femur = cfg.LIMB_PARAMETERS["l_femur"]
l_tibia_top = cfg.LIMB_PARAMETERS["l_tibia_top"]
l_tibia_bottom = cfg.LIMB_PARAMETERS["l_tibia_bottom"]
theta_tb = cfg.LIMB_PARAMETERS["theta_tibia_bend"]
theta_tt = cfg.LIMB_PARAMETERS["theta_tibia_top"]


# = Find r values where z_leg < z_root (z=0 @ ground) = #
r = symbols('r')

r_knee = 0
z_knee = (cfg.z_root+cfg.z_bodyClearance)

all_domains = []


for theta_k, theta_a in zip(theta_knee,theta_ankle):
    r_intervals = []

    r_ankle = r_knee + l_femur * DegMath.cos(theta_k)
    z_ankle = z_knee+ l_femur * DegMath.sin(theta_k)

    r_tibBend = r_ankle + l_tibia_top * DegMath.cos(theta_k + theta_a + theta_tt)
    z_tibBend = z_ankle + l_tibia_top * DegMath.sin(theta_k + theta_a + theta_tt)

    r_foot = r_tibBend + l_tibia_bottom * DegMath.cos(theta_k + theta_a + theta_tt + theta_tb)
    z_foot = z_tibBend + l_tibia_bottom * DegMath.sin(theta_k + theta_a + theta_tt + theta_tb)
    

    z_leg = Piecewise(
        (((DegMath.tan(theta_k) * (r - r_knee)) + z_knee), And( r> Min(r_knee,r_ankle) , r<= Max(r_knee,r_ankle) ) ), # Femur
        (((DegMath.tan(theta_k + theta_a + theta_tt) * (r - r_ankle)) + z_ankle), And( r> Min(r_ankle,r_tibBend) , r<= Max(r_ankle,r_tibBend) ) ), # Tibia Top
        (((DegMath.tan(theta_k + theta_a + theta_tt + theta_tb) * (r - r_tibBend)) + z_tibBend), And( r> Min(r_tibBend,r_foot) , r<= Max(r_tibBend,r_foot) )) #Tibia Bottom
    )



    domain = S.EmptySet

    for expr, cond in z_leg.args:

        sol = solveset(Lt(expr, cfg.z_root), r, domain=S.Reals)
    
        if isinstance(cond, And):
            cond_domain = S.Reals
            for subcond in cond.args:
                cond_domain = cond_domain.intersect(solveset(subcond, r, domain=S.Reals))
        else:
            cond_domain = solveset(cond, r, domain=S.Reals)

        domain_piece = sol.intersect(cond_domain)
        domain = domain.union(domain_piece)
        
    

    # Extract end points of domains
    if domain == S.EmptySet:
        pass  # No valid intervals
    elif isinstance(domain, Union):
        for interval in domain.args:
            if isinstance(interval, Interval):
                r_intervals.append([float(interval.start), float(interval.end)])
    elif isinstance(domain, Interval):
        r_intervals.append([float(domain.start), float(domain.end)])

    all_domains.append(r_intervals)
    
    
fig, ax = plt.subplots(figsize=(8, 8))

for i in range(len(r_range)):
    ax.plot([[all_domains[i][0][0]],[all_domains[i][0][1]]], [r_range[i],r_range[i]])
    ax.plot(r_range[i],r_range[i], 'ko' )
ax.plot([all_domains[0][0][1],all_domains[-1][0][0]],[r_range[0], r_range[-1]])

plt.show()
print([all_domains[0][0][1]])
print([all_domains[-1][0][0]])