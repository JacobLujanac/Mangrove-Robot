import json
import time
import serial

def send_joint_trajectories(leg_order, joint_trajs, duration, serial_port="/dev/ttyACM0", baudrate=115200):
    """
    leg_order: ["RF", "RM", ...] (must match order of joint_trajs)
    joint_trajs: list of arrays, one per leg. Each array = list of [hip, knee, ankle] over time
    duration: total duration in seconds
    """

    assert all(len(traj) == len(joint_trajs[0]) for traj in joint_trajs), "All legs must have same # of steps"

    payload = {
        "trajs": joint_trajs,        # List of [ [θ1, θ2, θ3], ... ] for each leg
        "legs": leg_order,           # Order of legs matching the above
        "duration": duration
    }

    # Send payload over serial
    ser = serial.Serial(serial_port, baudrate, timeout=1)
    time.sleep(0.5)  # let serial connect

    ser.write((json.dumps(payload) + "\n").encode("utf-8"))

    print("Sent trajectory")
    while True:
        line = ser.readline().decode("utf-8").strip()
        if line:
            print("Response:", line)
            if line == "Done":
                break

    ser.close()
