
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
def run_leg_optimizer(kneeROM, ankleROM,z_bodyClearance, run_index):
    # Input Parameters
    z_root = 20  # cm





    theta1_range_deg = np.linspace(kneeROM[0], kneeROM[1], 500)
    theta2_min, theta2_max = ankleROM[0] * (np.pi / 180), ankleROM[1] * (np.pi / 180)
    r_min = 1 #cm
    Tau = 35 # Motor Torque (kg-cm)
    W = 4 # Weight (kg) 4.5
    SF = 1 # Saftey Factor (-) 1.3





    # Constants
    deg2rad = np.pi / 180
    r_max = Tau / ( (1/3)*W*SF)
    z_samples = 25  # Reduced for speed
    z_vals = np.linspace(-z_bodyClearance, -z_bodyClearance+z_root, z_samples)


    # Limb lengths
    l1_values = np.linspace(3, r_max, 40)  # femur lengths
    l2_values = np.linspace(3, r_max, 40)  # tibia lengths

    W_values = np.zeros((len(l1_values), len(l2_values)))

    theta1_min, theta1_max = np.radians(kneeROM)
    theta2_min, theta2_max = np.radians(ankleROM)

    def is_reachable_vectorized(l1, l2, r, z_vals):
        dist = np.sqrt(r ** 2 + z_vals ** 2)
        if np.any(dist > (l1 + l2)) or np.any(dist < abs(l1 - l2)):
            return False

        dist_sq = r ** 2 + z_vals ** 2
        cos_theta2 = (dist_sq - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)

        if not np.all(np.abs(cos_theta2) <= 1):
            return False

        theta2_options = np.arccos(cos_theta2)
        reachable_all = np.ones(len(z_vals), dtype=bool)

        for sign in [1, -1]:
            t2 = sign * theta2_options
            k1 = l1 + l2 * np.cos(t2)
            k2 = l2 * np.sin(t2)
            theta1 = np.arctan2(z_vals, r) - np.arctan2(k2, k1)

            valid_joint_limits = (
                (theta1_min <= theta1) & (theta1 <= theta1_max) &
                (theta2_min <= t2) & (t2 <= theta2_max)
            )
            reachable_all &= valid_joint_limits

        return np.all(reachable_all)

    # Optimization loop
    W_values = np.zeros((len(l1_values), len(l2_values)))
    for i, l1 in enumerate(tqdm(l1_values, desc=f"Run {run_index} - Femur Scan")):
        for j, l2 in enumerate(l2_values):
            r_vals = np.linspace(r_min, r_max, 150)
            valid_r = [r for r in r_vals if is_reachable_vectorized(l1, l2, r, z_vals)]
            if valid_r:
                W_values[i, j] = max(valid_r) - min(valid_r)

    # Get results
    max_idx = np.unravel_index(np.argmax(W_values), W_values.shape)
    optimal_l1 = l1_values[max_idx[0]]
    optimal_l2 = l2_values[max_idx[1]]
    max_W = W_values[max_idx]
    r_vals2 = np.linspace(r_min, r_max, 150)
    valid_r2 = [r for r in r_vals2 if is_reachable_vectorized(optimal_l1, optimal_l2, r, z_vals)]
    ans_r_min = min(valid_r2) if valid_r2 else None
    ans_r_max = max(valid_r2) if valid_r2 else None

    # Plot
    L1, L2 = np.meshgrid(l1_values, l2_values, indexing='ij')
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(L1, L2, W_values, cmap='plasma')
    ax.set_xlabel(r'$l_{femur}$  [cm]')
    ax.set_ylabel(r'$l_{tibia}$ [cm]')
    ax.set_zlabel('Reachable Width [cm]')
    fig.suptitle(f'Optimized Leg Dimensions (Body Clearance = {z_bodyClearance} cm', fontsize=16)

    # ✨ Safe formatting
    r_range_text = (
        f"{ans_r_min:.2f}–{ans_r_max:.2f} cm"
        if ans_r_min is not None and ans_r_max is not None
        else "Unreachable"
    )

    ax.set_title(
        f"l1 = {optimal_l1:.2f} cm, l2 = {optimal_l2:.2f} cm, "
        f"r_range = {max_W:.2f} cm ({r_range_text})",
        fontsize=10
    )

    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)
    plt.tight_layout()
    plt.show()

    return {
        'run_index': run_index,
        'kneeROM': kneeROM,
        'ankleROM': ankleROM,
        'l1': optimal_l1,
        'l2': optimal_l2,
        'W': max_W,
        'r_min': ans_r_min,
        'r_max': ans_r_max
    }
# === RUN ALL 6 COMBINATIONS ===
# A = [(-135, 45), (-90, 90), (-45, 135)]
# B = [(-135, 135)]
# A = [(-90, 90)]
# B = [(-135,135)]


A = [-45,175]
B = [-175,180]

z_bC = np.arange(1,20,1)


results = []
run_id = 1
# for a in A:
#     for b in B:
#         results.append(run_leg_optimizer(a, b, z_bC, run_id))  # knee=A, ankle=B
#         run_id += 1
#         # results.append(run_leg_optimizer(b, a, run_id))  # knee=B, ankle=A
#         # run_id += 1

# for i in z_bC:
#     results.append(run_leg_optimizer(A, B, z_bC[i], run_id))
#     run_id += 1
results.append(run_leg_optimizer(A, B, 5, run_id))
results