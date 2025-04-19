import sys, ujson, select, time
from robot import Robot
from TalkWithBrain import send_touch_data  # assumed function

robot = Robot()

def run_joint_trajectories(joint_trajs, leg_names, duration):
    leg_objs = robot.get_leg_objects(leg_names)
    steps = len(joint_trajs[0])
    assert all(len(traj) == steps for traj in joint_trajs)

    step_time = duration / steps
    start_time = time.ticks_ms()

    for i in range(steps):
        target_time = time.ticks_add(start_time, int(i * step_time * 1000))
        for traj, leg in zip(joint_trajs, leg_objs):
            leg.set_angles(*traj[i])
        while time.ticks_diff(target_time, time.ticks_ms()) > 0:
            time.sleep_ms(1)

    send_touch_data()

while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline()
            if line:
                payload = ujson.loads(line)
                print("Received:", payload)
                run_joint_trajectories(payload["trajs"], payload["legs"], payload["duration"])
                print("Done")
        except Exception as e:
            print("Error:", e)
