import matplotlib.pyplot as plt
import random, json, os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as cfg

# === PARAMETERS ===
n = 14               # Grid rows
m = 23               # Grid columns
d = 2.8              # Node spacing in centimeters
rho_root = 0.01      # Root density (roots/cm^2)
P_root = rho_root * n * m * (d ** 2)
P_rootTall = 0       # % of roots that are tall (update to 0.1 when implementing)
N = 1                # Number of targets


# === GENERATOR FUNCTION ===
def generate_grid(n, m, d, P_root, P_rootTall, N):
    root_points = []
    tallRoot_points = []
    empty_points = []
    target_points = []
    all_coords = []

    for i in range(n):
        for j in range(m):
            x = i * d
            y = j * d
            all_coords.append((x, y))

    # Place N target points in upper quarter of board
    board_height = (m - 1) * d
    targ_dist_min = board_height * 0.85
    eligible_coords = [coord for coord in all_coords if coord[1] > targ_dist_min]

    for _ in range(N):
        target_loc = random.choice(eligible_coords)
        eligible_coords.remove(target_loc)
        all_coords.remove(target_loc)
        target_points.append(target_loc)

    # Randomly assign root vs empty
    for (x, y) in all_coords:
        roll = random.uniform(0, 100)
        if roll < P_rootTall:
            tallRoot_points.append((x, y))
        elif roll < P_root:
            root_points.append((x, y))
        else:
            empty_points.append((x, y))

    return {
        "root_points": root_points,
        "tallRoot_points": tallRoot_points,
        "empty_points": empty_points,
        "target_points": target_points,
        "spacing": d,
        "grid_size": (n, m)
    }


# === SAVE FUNCTION ===
def save_map_data(map_data, json_path="map_data.json", image_path="mangrove_map.png"):
    # Save JSON
    serializable_data = {
        k: [list(pt) for pt in v] if isinstance(v, list) else v
        for k, v in map_data.items()
    }
    with open(json_path, "w") as f:
        json.dump(serializable_data, f, indent=4)

    # Save Plot
    plt.figure(figsize=(8, 8))
    if map_data["root_points"]:
        rx, ry = zip(*map_data["root_points"])
        rootHandle = plt.scatter(rx, ry, color='black', marker='^',label='Root')
    if map_data["tallRoot_points"]:
        px, py = zip(*map_data["tallRoot_points"])
        tallRootHandle = plt.scatter(px, py, color='purple', label='Tall Root')
    if map_data["empty_points"]:
        ex, ey = zip(*map_data["empty_points"])
        plt.scatter(ex, ey, color='gray', alpha=0.2, label='Empty', s=10)
    if map_data["target_points"]:
        tx, ty = zip(*map_data["target_points"])
        targHandle = plt.scatter(tx, ty, color= cfg.color_dict["Target"], marker='*', s=600, label='Target')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend(handles = [targHandle,rootHandle], labels = ["Target","Root"], loc="upper left", bbox_to_anchor=(1,1))  # Moves legend outside top right
    plt.title(f"{round(n*d,3)}cm x {round(m*d,3)}cm Mangrove Simulation")
    plt.xticks([])
    plt.yticks([])

    plt.savefig(image_path, bbox_inches="tight")
    plt.show()


# === RUN SCRIPT ===
if __name__ == "__main__":
    map_data = generate_grid(n, m, d, P_root, P_rootTall, N)
    save_map_data(map_data)
