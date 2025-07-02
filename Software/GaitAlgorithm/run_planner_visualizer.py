import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import Software.Core.config as config
from GaitAlgorithm import planner_result
from algorithmVisualizer import visualize_gait
from visual_utils import *



tripods = config.TRIPOD_LEGS


# Run the gait planner and visualize the result

visualize_gait(planner_result, use_tripod= "B")
draw_tripod_leg_roms_from_result(planner_result, tripod_name="B")

# === Visualize the viable translation zone ===
draw_viable_translation_zone(
    planner_result["foot_positions"],
    tripods[planner_result["tripod"]],
    planner_result["viable_zone"],planner_result["robot_pose"])



plt.show()