import serial
import time

# === Define your ports ===
motor_port = 'COM6'        # Arduino handling X/Y motorclear
pressure_port = 'COM5'  # Arduino handling M/R/Z

# === Connect to both Arduinos ===
motor_ser = serial.Serial(motor_port, 57600, timeout=1)
pressure_ser = serial.Serial(pressure_port, 57600, timeout=1)
time.sleep(2)

print("Ready.")
print("• Use X0–X4 and Y0–Y6 for motor control")
print("• Use M<number>, R, Z for pressure control")
print("• Type 'exit' to quit\n")

try:
    while True:
        cmd = input("Enter command: ").strip().upper()
        if cmd == 'EXIT':
            break

        if not cmd:
            print("Empty command.")
            continue

        # Determine which Arduino to send to
        target = None
        if cmd.startswith("X") and cmd[1:].isdigit():
            target = 'motor'
        elif cmd.startswith("Y") and cmd[1:].isdigit():
            target = 'motor'
        elif cmd.startswith("M") and cmd[1:].isdigit():
            target = 'pressure'
        elif cmd in ["R", "Z"]:
            target = 'pressure'
        else:
            print("Invalid command. Use X#, Y#, M#, R, or Z.")
            continue

        # Choose serial port
        ser = motor_ser if target == 'motor' else pressure_ser

        # Send command
        ser.write((cmd + '\n').encode())
        print(f"[Python → {target}] Sent {cmd}")

        # Wait for response
        while True:
            line = ser.readline().decode().strip()
            if line:
                print(f"[{target.upper()} Arduino] {line}")
                if (
                    line.startswith("DoneX:") or
                    line.startswith("DoneY:") or
                    line.startswith("DoneM:") or
                    line.startswith("Pressure:") or
                    "reset" in line or
                    "Error" in line
                ):
                    break

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    motor_ser.close()
    pressure_ser.close()
    print("Serial connections closed.")