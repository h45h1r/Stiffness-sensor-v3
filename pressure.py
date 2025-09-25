import serial
import time

# Adjust this if your port changes
ser = serial.Serial('/dev/tty.usbmodem113401', 57600, timeout=1)
time.sleep(2)

print("Ready. Type commands like M25, R, Z. Type 'exit' to quit.\n")

try:
    while True:
        cmd = input("Enter command: ").strip().upper()
        if cmd == 'EXIT':
            break
        if not cmd or (cmd[0] not in ['M', 'R', 'Z']):
            print("Invalid command. Use M<number>, R, or Z.")
            continue

        # Send command with newline
        ser.write((cmd + '\n').encode())
        print(f"[Python] Sent {cmd}")

        while True:
            line = ser.readline().decode().strip()
            if line:
                print(f"[Arduino] {line}")
                # Stop reading when we get a final confirmation
                if line.startswith("DoneM:") or line.startswith("Pressure:") or "reset" in line:
                    break

except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    ser.close()
    print("Serial connection closed.")