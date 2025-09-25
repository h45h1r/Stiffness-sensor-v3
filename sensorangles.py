import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/tty.usbmodem101', 57600, timeout=1)
time.sleep(2)  # Allow Arduino to reset

print("Ready. Type X0–X4 or Y0–Y6 to move motors. Type 'exit' to quit.\n")

try:
    while True:
        cmd = input("Enter command: ").strip().upper()
        if cmd == 'EXIT':
            break
        if not (cmd.startswith("X") or cmd.startswith("Y")):
            print("Invalid command. Use format like X2 or Y3.")
            continue

        ser.write((cmd + "\n").encode())
        print(f"[Python] Sent {cmd}")

        while True:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino] {line}")
                if line.startswith("DoneX:") or line.startswith("DoneY:") or "Error" in line:
                    break

except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    ser.close()
    print("Serial connection closed.")