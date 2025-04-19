# brain_controller.py
import serial
import json
import time

# Set to your Servo2040's serial device
SERVO2040_PORT = "/dev/ttyACM0"
BAUDRATE = 115200

def send_command(ser, cmd, args=None):
    message = {
        "cmd": cmd,
        "args": args or {}
    }
    ser.write((json.dumps(message) + "\n").encode())

def wait_for_done_or_sensor(ser):
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
            if msg.get("type") == "touch":
                print("Touch data:", msg["data"])
            elif msg.get("type") == "status" and msg.get("message") == "done":
                print("Movement completed.")
                break
            else:
                print("Other message:", msg)
        except Exception as e:
            print("Error parsing message:", e)


def main():
    ser = serial.Serial(SERVO2040_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Give time for Pico to reset if just plugged in

    # # Example movement: 2 legs, 2 steps, 3 joints per leg
    # move_args = {
    #     "legs": [[0, 1, 2], [3, 4, 5]],  # Motor IDs per leg
    #     "trajs": [
    #         [[45, 30, 60], [50, 35, 62]],
    #         [[90, 60, 70], [92, 62, 74]]
    #     ],
    #     "duration": 1.2  # seconds
    # # }

    # send_command(ser, "move", move_args)
    # wait_for_done(ser)

if __name__ == "__main__":
    main()
