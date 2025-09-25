import nidaqmx
from nidaqmx.constants import TerminalConfiguration
import matplotlib.pyplot as plt
import time
from collections import deque

# === CONFIGURATION ===
DAQ_CHANNEL = "Dev1/ai0"         # Update this if needed
SENSITIVITY_V_PER_N = 0.1        # 1 N = 0.1 V → 10 N per 1V → change based on your sensor!
MAX_POINTS = 100                 # Number of points to show in the plot

# === SETUP ===
voltages = deque(maxlen=MAX_POINTS)
forces = deque(maxlen=MAX_POINTS)
timestamps = deque(maxlen=MAX_POINTS)

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], label="Force (N)")
ax.set_ylim(-10, 10)
ax.set_xlim(0, MAX_POINTS)
ax.set_xlabel("Samples")
ax.set_ylabel("Force (N)")
ax.legend()
plt.title("Live Force Sensor Readings")

# === FUNCTION ===
def voltage_to_force(voltage):
    return voltage / SENSITIVITY_V_PER_N

def update_plot(force):
    forces.append(force)
    timestamps.append(time.time())
    line.set_ydata(forces)
    line.set_xdata(range(len(forces)))
    ax.relim()
    ax.autoscale_view()
    plt.draw()
    plt.pause(0.01)

# === MAIN LOOP ===
def main():
    print(f"Reading from {DAQ_CHANNEL}...")
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan(
            DAQ_CHANNEL,
            terminal_config=TerminalConfiguration.RSE
        )
        try:
            while True:
                voltage = task.read()
                force = voltage_to_force(voltage)
                print(f"Voltage: {voltage:.5f} V → Force: {force:.2f} N")
                update_plot(force)
        except KeyboardInterrupt:
            print("\nStopped by user.")

if __name__ == "__main__":
    main()
