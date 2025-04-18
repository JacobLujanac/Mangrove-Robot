import matplotlib.pyplot as plt
import random

# Grid Dimensions
n = 14
m = 23

# Node Spacing
d = 28 / 1000 #m

# Root Densities
rho_root = 100 #roots / m^2

P_root = rho_root * n * m * (d**2)
print(P_root)
P_rootTall = P_root * 0.1

# Number of targets
N = 2


def generate_grid(n, m, d, P_root, P_rootTall, N):
    red_points = []
    purple_points = []
    empty_points = []
    target_points = []
    all_coords =[]

    for i in range(n):
        for j in range(m):
            x = i * d
            y = j * d
            all_coords.append((x, y))

    # Place N amount of targets randomly
    for i in range(N):   
        target_points = random.choice(all_coords)
        all_coords.remove(target_points)

    # Randomly place roots
    for (x,y) in all_coords:
            roll = random.uniform(0, 100)
            if roll < P_rootTall:
                purple_points.append((x, y))
            elif roll < P_root:
                red_points.append((x, y))
            else:
                empty_points.append((x, y))

    # Plotting
    plt.figure(figsize=(8, 8))
    if red_points:
        rx, ry = zip(*red_points)
        plt.scatter(rx, ry, color='red', label='Root')
    if purple_points:
        px, py = zip(*purple_points)
        plt.scatter(px, py, color='purple', label='Tall Root')
    if empty_points:
        ex, ey = zip(*empty_points)
        plt.scatter(ex, ey, color='gray', alpha=0.2, label='Empty', s=10)
    if target_points:
        plt.scatter(*target_points, color='green', s=200, label='Target')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    plt.title(f"{round(n*(d*100),3)}cm x{round(m* (d*100),3)}cm Mangrove Simulation")
    plt.grid(True)
    plt.show()

generate_grid(n, m, d, P_root, P_rootTall,N)
