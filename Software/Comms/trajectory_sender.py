import json
import time
#import serial

class Servo2040Interface:
    def __init__(self, serial_port="/dev/ttyACM0", baudrate=115200):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = None

    def connect(self):
        # === Establish Serial Connection to Servo2040 === #
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        time.sleep(0.5)  # Allow time for Servo2040 to reset if needed
        print("[Pi] Connected to Servo2040.")
        
 
#
        # === Trigger MicroPython Soft Reboot === #
        print("[Pi] Sending soft reboot (Ctrl-D)...")
        self.ser.write(b'\x04')  # Ctrl-D
        self.ser.flush()
        time.sleep(1)
        self.ser.write(b'\r\n')   # Wake up
        self.ser.write(b"import main\r\n")  # Manually run main.py
        self.ser.flush()
        time.sleep(1)  # Allow time to fully load



        
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[Pi] Serial port closed.")

    def send_joint_trajectories(self, legsMoving, joint_trajs, duration):    
        assert all(len(traj) == len(joint_trajs[0]) for traj in joint_trajs), "All legs must have same # of steps"
        

        # === Prepare Payload ===
        joint_trajs_serializable = [traj.tolist() for traj in joint_trajs]
        payload = {
            "legs": legsMoving,
            "trajs": joint_trajs_serializable,
            "duration": duration
        }

        # === Send Payload ===
        payload_json = json.dumps(payload) + "\n"
        self.ser.write(payload_json.encode("utf-8"))
        self.ser.flush()
        print("[Pi] Sent trajectory payload.")

        final_joint_angles = []
        start_time = time.time()
        timeout_seconds = 15

        print("[Pi] Listening for Servo2040 responses...")

        while True:
            if self.ser.in_waiting:
                # Read one line
                line = self.ser.readline().decode('utf-8', errors="replace").strip()
                if not line:
                    continue

                print(f"[Pi][Servo2040 Raw] {line}")  # Print all incoming data

                try:
                    # Try parsing as JSON
                    data = json.loads(line)
                    if isinstance(data, dict):
                        if data.get("type") == "final_angles":
                            print("[Pi] Received final angles:")
                            for leg_info in data["data"]:
                                print(f"  {leg_info['leg']}: {leg_info['angles']}")
                            final_joint_angles.append(data)
                        else:
                            print(f"[Pi] Received unknown JSON message: {data}")
                except json.JSONDecodeError:
                    # If not JSON, check for specific keywords
                    if line.lower() == "done":
                        print("[Pi] Trajectory execution complete.")
                        break
                    # Otherwise just a normal message (already printed above)

            if time.time() - start_time > timeout_seconds:
                print("[Pi] Timeout waiting for Servo2040 response.")
                break

        return final_joint_angles



