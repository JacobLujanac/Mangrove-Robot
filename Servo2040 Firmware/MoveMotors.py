import sys
import ujson
import TalkWithBrain


def run_joint_trajectories(joint_trajs, motor_ids_grouped, duration):
            """
        joint_trajs: list of [steps x 3] angle arrays, one per leg
        leg_objs: list of leg objects
        duration: total motion time in seconds
        """
        steps = len(joint_trajs[0])
        assert all(len(traj) == steps for traj in joint_trajs)

        step_time = duration / steps
        start_time = time.ticks_ms()

        for i in range(steps):
            # Calculate target time using ticks (handles overflow safely)
            target_time = time.ticks_add(start_time, int(i * step_time * 1000))

            # Set angles for each leg
            for angles, leg in zip(joint_trajs, leg_objs):
                leg.set_angles(*angles[i])

            # Wait until it's time for the next step
            while time.ticks_diff(target_time, time.ticks_ms()) > 0:
                time.sleep_ms(1)  # don’t waste CPU cycles
                
        # Touch Sensor Status
        send_touch_data()
    pass

while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline()
            if line:
                payload = ujson.loads(line)
                print("Received:", payload)

                run_joint_trajectories(
                    payload["trajs"],
                    payload["legs"],
                    payload["duration"]
                )
                print("Done")  # can be used as Pi-side signal

        except Exception as e:
            print("Error:", e)
