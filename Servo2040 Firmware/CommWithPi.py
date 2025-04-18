# main.py
import sys
import ujson as json
import time
import select


# Touch Sensor Info
# Example: 6 touch sensors on digital pins
touch_pins = [Pin(i, Pin.IN, Pin.PULL_DOWN) for i in (10, 11, 12, 13, 14, 15)]
# Dummy ROM limits (degrees)
motor_rom = {
    0: (0, 180), 1: (0, 180), 2: (0, 180),
    3: (0, 180), 4: (0, 180), 5: (0, 180),
}



def read_touch_sensors():
    return [pin.value() for pin in touch_pins]

def send_touch_data():
    data = {
        "type": "touch",
        "data": read_touch_sensors()
    }
    print(json.dumps(data))


def handle_command(msg):
    cmd = msg.get("cmd")
    args = msg.get("args", {})

    if cmd == "move":
        run_joint_trajectories(
            args["trajs"],
            args["legs"],
            args["duration"]
        )
    elif cmd == "ping":
        print("pong")
    else:
        print(f"Unknown command: {cmd}")

while True:
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline()
            if line:
                handle_command(json.loads(line))
        except Exception as e:
            print("Error parsing:", e)

