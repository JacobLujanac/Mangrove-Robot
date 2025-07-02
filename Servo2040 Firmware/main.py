import sys, ujson, select, time, config
from MotorClass_Servo2040 import Motor
from LegClass_Servo2040 import Leg

print("[Servo2040] Booted")

# === SETUP LEGS === #
legs = []
LEG_NAMES = config.LEG_NAMES

for i in range(len(LEG_NAMES)):
    name = LEG_NAMES[i]
    servo_indices = [i*3, i*3 + 1, i*3 + 2]
    side = "Left" if name[0] == "L" else "Right"
    legs.append(Leg(name, servo_indices, side))

# === Helper === #
def get_leg_by_name(name):
    for leg in legs:
        if leg.name == name:
            return leg
    raise ValueError(f"[Servo2040 Error] Leg with name '{name}' not found")

# === RUNNER === #
def run_joint_trajectories(joint_trajs, leg_names, duration):
    print("[Servo2040] Starting run_joint_trajectories()")
    step_count = len(joint_trajs[0])
    assert all(len(traj) == step_count for traj in joint_trajs)
    step_time = duration / step_count
    start_time = time.ticks_ms()

    for i in range(step_count):
        target_time = time.ticks_add(start_time, int(i * step_time * 1000))
        for traj, name in zip(joint_trajs, leg_names):
            try:
                leg = get_leg_by_name(name)
                angles = traj[i]
                leg.set_angles(*angles)
            except Exception as e:
                print("[Servo2040 Error] during set_angles:", e)
        while time.ticks_diff(target_time, time.ticks_ms()) > 0:
            time.sleep_ms(1)

    # === Prepare Final Data === #

    # = Joint Angles = #
    print("[Servo2040] Preparing final joint angles...")
    joint_data = []
    for name in leg_names:
        try:
            leg = get_leg_by_name(name)
            joint_data.append({
                "leg": name,
                "angles": leg.get_angles()
            })
        except Exception as e:
            print("[Servo2040 Error] during get_angles:", e)
    
    # = Touch Sensor Status = #
    touch_status = {}
    for name in leg_names:
        try:
            leg = get_leg_by_name(name)
            touch_status[name] = leg.is_touched()
        except Exception as e:
            print("[Servo2040 Error] during touch check:", e)
# === Send Results === #
print("[Servo2040] Sending final data...")
print(ujson.dumps({
    "type": "final_angles",
    "data": joint_data,
    "touch_data": touch_status
}))
print("[Servo2040] Sending Done...")
print("Done")

# === MAIN LOOP === #
print("[Servo2040] Ready and waiting for commands...")

while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline().strip()
            print("[Servo2040] Raw line received:", line)

            if line.startswith("{"):
                try:
                    payload = ujson.loads(line)
                    print("[Servo2040] Parsed payload:", payload)

                    run_joint_trajectories(payload["trajs"], payload["legs"], payload["duration"])
                    
                except Exception as parse_error:
                    print("[Servo2040 Error] JSON Parse Error:", parse_error)
            else:
                print("[Servo2040] Ignored non-JSON line.")

        except Exception as e:
            print("[Servo2040 Error]", e)

    time.sleep(0.01)




