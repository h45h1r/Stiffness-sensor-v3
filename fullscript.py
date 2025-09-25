import serial
import time
import timeit
import numpy as np
import csv
import re
from threading import Event
import nidaqmx
import math
try:
    import cv2
    CV2_AVAILABLE = True
    CAMERA_BACKEND = cv2.CAP_DSHOW  # Use the backend that worked in your test
except ImportError:
    print("Warning: OpenCV (cv2) not available. Image capture will be disabled.")
    CV2_AVAILABLE = False
    CAMERA_BACKEND = None
import os
from datetime import datetime

# ========== SETTINGS ==========
np.set_printoptions(formatter={'float_kind': '{:f}'.format})
# CNC Coordinates (X, Y, Z)
x_cord = -146
y_cord = -45
z_cord = -6.220  # Initial guess — actual contact Z will be found dynamically

# CNC Serial Setup
s_machine = serial.Serial('COM4', 115200)

# Arduino Serial Ports (optional)
motor_port = 'COM6'        # Arduino handling X/Y
pressure_port = 'COM5'     # Arduino handling M/R/Z

# Camera settings
camera_index = 1  # Use the working camera index
image_delay = 2.0  # Delay after positioning before taking image (seconds)

# Global variables for sensor data
calibration_matrix_trimmed = None
bias_reading = None
data_from_DAQ = None
contact_threshold_z = None

# Global variables for interactive control
current_x = x_cord
current_y = y_cord
current_z = 0
current_x_angle = 0.0  # Track current X angle
current_y_angle = 0.0  # Track current Y angle
current_target_pressure = 0.0  # Track current target pressure setting
base_radius = 20  # Radius = diameter/2 = 40mm/2 = 20mm
safe_z_value = 0  # Safe Z height above surface (adjust as needed)
contact_z_position = None  # Store the found contact Z position
z_safe_offset = 0.1  # Safe offset above contact position (mm)
last_contact_z = None  # Store the last contact Z position for Y commands

# Arduino angle arrays (matching your Arduino code)
thetaXAngles = [0, 5.625, 11.25, 16.875, 22.5]  # X0, X1, X2, X3, X4
thetaYAngles = [0, 5.625, 11.25, 16.875, 22.5, 28.125, 33.75]  # Y0, Y1, Y2, Y3, Y4, Y5, Y6

# Experiment parameters
pressure_values = [0, 1, 2, 3, 4]  # M0=0psi, M1=0.29psi, M2=0.58psi, M3=0.87psi, M4=1.16psi
pressure_mapping = {0: 0.0, 1: 0.29, 2: 0.58, 3: 0.87, 4: 1.16}  # Map indices to actual pressure values in psi

# Displacement configuration (easily adjustable)
DISPLACEMENT_CONFIG = {
    'images_per_step': 100,      # Number of images per 0.1mm step (can be 50 or 100 later)
    'increment': 0.1,          # mm per step
    'max_displacement': 2.1,   # mm total displacement
    'step_delay': 1.0          # seconds between steps
}

# ========== SENSOR FUNCTIONS ==========
import xml.etree.ElementTree as ET

def read_calibration_file():
    """
    Parse ATI Mini40 .cal XML and build a 6x6 calibration matrix from <UserAxis>.
    Order: Fx, Fy, Fz, Tx, Ty, Tz. Each 'values' attribute has 6 numbers.
    """
    cal_file = os.path.join(os.path.dirname(__file__), "FT26196.cal")
    try:
        # Read and sanitize XML to tolerate trailing/invalid bytes
        with open(cal_file, 'rb') as f:
            raw_bytes = f.read()

        # Strip UTF-8 BOM if present
        if raw_bytes.startswith(b"\xef\xbb\xbf"):
            raw_bytes = raw_bytes[3:]

        # Keep only up to and including the closing root tag
        end_tag = b"</FTSensor>"
        end_idx = raw_bytes.find(end_tag)
        if end_idx != -1:
            raw_bytes = raw_bytes[: end_idx + len(end_tag)]

        # Remove disallowed control chars (keep tab/newline/carriage return)
        import re as _re
        raw_bytes = _re.sub(rb"[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]", b"", raw_bytes)

        # Decode to text for XML parser
        xml_text = raw_bytes.decode('utf-8', errors='ignore')

        # Parse from string (more tolerant than ET.parse for odd file encodings)
        root = ET.fromstring(xml_text)

        # Find the Calibration node
        calib = root.find("Calibration")
        if calib is None:
            raise ValueError("No <Calibration> element found in calibration file")

        axis_order = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
        rows = []

        for name in axis_order:
            ua = calib.find(f'UserAxis[@Name="{name}"]')
            if ua is None:
                raise ValueError(f'Missing <UserAxis Name="{name}"> in calibration file')
            # Split by whitespace, convert to float (there should be 6)
            vals = [float(x) for x in ua.attrib["values"].split()]
            if len(vals) != 6:
                raise ValueError(f'UserAxis {name} has {len(vals)} values, expected 6')
            rows.append(vals)

        matrix = np.array(rows, dtype=float)  # shape (6,6)

        # Expose globally for the rest of your pipeline
        global calibration_matrix_trimmed
        calibration_matrix_trimmed = matrix

        print(f"Calibration file loaded successfully. Matrix shape: {calibration_matrix_trimmed.shape}")

    except FileNotFoundError:
        print(f"Warning: Calibration file {cal_file} not found. Sensor functions may not work.")
        calibration_matrix_trimmed = np.eye(6)
    except Exception as e:
        print(f"Error reading calibration file: {e}")
        calibration_matrix_trimmed = np.eye(6)

def sensor_bias():
    global bias_reading
    bias_reading = np.zeros(6)
    
    try:
        print("Calculating sensor bias...")
        for i in range(10):
            get_DAQ_data()
            if data_from_DAQ is not None:
                bias_reading += np.array(data_from_DAQ)
            time.sleep(0.1)
        bias_reading /= 10
        print("Sensor bias calculated successfully")
        
    except Exception as e:
        print(f"Error calculating sensor bias: {e}")
        bias_reading = np.zeros(6)

def get_DAQ_data():
    global data_from_DAQ
    try:
        with nidaqmx.Task() as task:
            for i in range(6):
                task.ai_channels.add_ai_voltage_chan(f"Dev1/ai{i}")
            data_from_DAQ = task.read()
    except Exception as e:
        print(f"Error reading DAQ data: {e}")
        data_from_DAQ = [0.0] * 6

def get_FT_data():
    try:
        get_DAQ_data()
        if data_from_DAQ is None or calibration_matrix_trimmed is None:
            return np.zeros(6)
        raw = np.array(data_from_DAQ)
        biased = raw - bias_reading
        calibrated = np.transpose(np.matmul(calibration_matrix_trimmed, biased.T))
        return calibrated
    except Exception as e:
        print(f"Error getting FT data: {e}")
        return np.zeros(6)

def initialize_ATI_mini40():
    print("Initializing ATI Mini40 sensor...")
    read_calibration_file()
    sensor_bias()
    print("ATI Mini40 sensor initialized")

# ========== PRESSURE CONTROL ==========
def initialize_pressure_control():
    """Initialize pressure control Arduino connection"""
    try:
        global pressure_ser
        pressure_ser = serial.Serial(pressure_port, 57600, timeout=1)
        time.sleep(2)
        print(f"Pressure control initialized on {pressure_port}")
        return True
    except Exception as e:
        print(f"Warning: Could not initialize pressure control on {pressure_port}: {e}")
        return False

def initialize_motor_control():
    """Initialize motor control Arduino connection"""
    try:
        global motor_ser
        motor_ser = serial.Serial(motor_port, 57600, timeout=1)
        time.sleep(2)
        print(f"Motor control initialized on {motor_port}")
        return True
    except Exception as e:
        print(f"Warning: Could not initialize motor control on {motor_port}: {e}")
        return False

def send_pressure_command(command):
    """Send pressure command to Arduino (M/R/Z)"""
    try:
        # Update global target pressure tracking
        global current_target_pressure
        if command.startswith('M') and len(command) > 1:
            try:
                pressure_index = int(command[1:])
                current_target_pressure = pressure_mapping.get(pressure_index, pressure_index)  # Use mapping or fallback to index
            except ValueError:
                pass
        elif command in ['R', 'Z']:
            current_target_pressure = 0.0
        
        if 'pressure_ser' in globals():
            pressure_ser.write((command + '\n').encode())
            print(f"Sent pressure command: {command}")
            
            # Wait for response
            timeout = time.time() + 5
            while time.time() < timeout:
                if pressure_ser.in_waiting:
                    response = pressure_ser.readline().decode().strip()
                    print(f"Pressure response: {response}")
                    if response.startswith("DoneM:") or response.startswith("Pressure:") or "reset" in response:
                        return True
            return True
        else:
            print("Pressure control not available - simulating pressure command")
            print(f"Simulated: {command}")
            return True  # Return True to allow program to continue
    except Exception as e:
        print(f"Error sending pressure command: {e}")
        print("Simulating pressure command to continue...")
        return True  # Return True to allow program to continue

def read_pressure_sensor():
    """Read current pressure value from Arduino pressure sensor"""
    try:
        if 'pressure_ser' in globals():
            pressure_ser.write(b'P\n')  # Send 'P' command to Arduino
            time.sleep(0.1)
            
            if pressure_ser.in_waiting:
                response = pressure_ser.readline().decode().strip()
                if response.startswith("PRESSURE:"):
                    pressure_value = float(response.split(":")[1])
                    return pressure_value
                else:
                    print(f"Unexpected pressure response: {response}")
                    return None
        else:
            print("Pressure control not available - cannot read pressure")
            return None
    except Exception as e:
        print(f"Error reading pressure: {e}")
        return None

def send_motor_command(command):
    """Send motor command to Arduino (X0-X4, Y0-Y6)"""
    try:
        if 'motor_ser' in globals():
            motor_ser.write((command + '\n').encode())
            print(f"Sent motor command: {command}")
            
            # Wait for response
            timeout = time.time() + 10
            while time.time() < timeout:
                if motor_ser.in_waiting:
                    response = motor_ser.readline().decode().strip()
                    print(f"Motor response: {response}")
                    if response.startswith("DoneX:") or response.startswith("DoneY:") or "Error" in response:
                        return True
            return True
        else:
            print("Motor control not available - simulating motor command")
            print(f"Simulated: {command}")
            return True  # Return True to allow program to continue
    except Exception as e:
        print(f"Error sending motor command: {e}")
        print("Simulating motor command to continue...")
        return True  # Return True to allow program to continue

def interactive_pressure_control():
    """Interactive pressure sensor adjustment"""
    print("\n=== PRESSURE SENSOR ADJUSTMENT ===")
    print("Available commands:")
    print("  M<number> - Set pressure to specific value (e.g., M3)")
    print("  R - Reset pressure")
    print("  Z - Zero pressure")
    print("  skip - Skip pressure adjustment")
    
    while True:
        command = input("\nEnter pressure command (M/R/Z/skip): ").strip().upper()
        
        if command == 'SKIP':
            print("Skipping pressure adjustment")
            return True
        elif command == 'EXIT':
            return False
        elif command.startswith('M') and len(command) > 1:
            try:
                value = int(command[1:])
                if send_pressure_command(command):
                    print(f"Pressure set to {value}")
                    return True
            except ValueError:
                print("Invalid M command. Use format M<number>")
        elif command in ['R', 'Z']:
            if send_pressure_command(command):
                print(f"Pressure command {command} executed")
                return True
        else:
            print("Invalid command. Use M<number>, R, Z, or skip")

def return_to_zero_positions():
    """Return CNC to zero positions after pressure change"""
    print("\nReturning to zero positions...")
    
    # Move to zero positions at safe height
    if contact_z_position is not None:
        safe_z = contact_z_position + z_safe_offset
        if not move_to_position(x_cord, y_cord, safe_z):
            print("Failed to return to zero positions")
            return False
    else:
        if not move_to_position(x_cord, y_cord, safe_z_value):
            print("Failed to return to zero positions")
            return False
    
    print("Returned to zero positions successfully")
    return True

# ========== CNC CONTROL ==========
def wait_for_movement_completion():
    print("Waiting for movement completion...")
    time.sleep(1)
    idle_counter = 0
    max_attempts = 50
    attempts = 0
    
    while attempts < max_attempts:
        time.sleep(0.1)
        try:
            s_machine.reset_input_buffer()
            s_machine.write(b'?\n')
            s_machine.flush()
            
            start_time = time.time()
            response = ""
            while time.time() - start_time < 2:
                if s_machine.in_waiting:
                    response = s_machine.readline().strip().decode('utf-8')
                    break
                time.sleep(0.05)
            
            print(f"Status Check: {response}")
            
            if 'Idle' in response or 'Home' in response:
                idle_counter += 1
                if idle_counter > 3:
                    print("Movement completed")
                    break
            else:
                idle_counter = 0
                
        except Exception as e:
            print(f"Error checking status: {e}")
        attempts += 1
    
    if attempts >= max_attempts:
        print("Warning: Maximum wait attempts reached")

def send_command(command):
    print(f"Sending: {command}")
    try:
        s_machine.write((command + '\n').encode())
        s_machine.flush()
        time.sleep(0.1)
        
        timeout = time.time() + 10
        while time.time() < timeout:
            if s_machine.in_waiting:
                response = s_machine.readline().decode().strip()
                print(f"Response: {response}")
                if response == 'ok' or 'error' in response.lower():
                    if 'error' in response.lower():
                        print(f"ERROR: {response}")
                        return False
                    return True
        print("Warning: No response received")
        return False
        
    except Exception as e:
        print(f"Error sending command: {e}")
        return False

def initialize_cnc():
    print("Initializing CNC machine...")
    
    try:
        s_machine.write(b"\r\n\r\n")
        time.sleep(2)
        s_machine.reset_input_buffer()
        
        send_command('$X')
        time.sleep(0.5)
        
        print("Homing machine...")
        send_command('$H')
        wait_for_movement_completion()
        
        send_command('$$')
        send_command('G90')
        send_command('G21')
        
        feed_rate(1000, 1000, 100)
        print("CNC machine initialized successfully")
        return True
        
    except Exception as e:
        print(f"Error initializing CNC: {e}")
        return False

def feed_rate(X_feed, Y_feed, Z_feed):
    print(f"Setting feed rates: X={X_feed}, Y={Y_feed}, Z={Z_feed}")
    send_command(f'$110={X_feed}')
    send_command(f'$111={Y_feed}')
    send_command(f'$112={Z_feed}')

def move_to_position(x, y, z, feed_rate_xy=1000, feed_rate_z=100):
    print(f"Moving to position: X={x}, Y={y}, Z={z}")
    success = send_command(f'G0 X{x} Y{y} F{feed_rate_xy}')
    if not success:
        print("Failed to move X and Y axes")
        return False
    wait_for_movement_completion()
    
    success = send_command(f'G0 Z{z} F{feed_rate_z}')
    if not success:
        print("Failed to move Z axis")
        return False
    wait_for_movement_completion()
    
    # Update current position
    global current_x, current_y, current_z
    current_x, current_y, current_z = x, y, z
    
    print(f"Successfully moved to X={x}, Y={y}, Z={z}")
    return True

def calculate_x_from_theta(theta_degrees):
    """Calculate X position based on theta angle"""
    theta_radians = math.radians(theta_degrees)
    # Use sine for X movement - increased scaling for more movement
    x_offset = base_radius * math.sin(theta_radians) * 4.60#.25  # Increased from 0.5 to 1.0
    new_x = x_cord + x_offset
    print(f"X offset calculation: angle={theta_degrees}°, offset={x_offset:.3f}mm, new_x={new_x:.3f}")
    return new_x

def find_contact_point(x, y, z_start=None, z_limit=-30, contact_threshold=-0.03):
    """Find contact point at given X,Y position with fast approach then precise detection"""
    print(f"\nFinding contact point at X={x}, Y={y}")
    print("Fast approach to sensor vicinity...")
    
    if z_start is None:
        z_start = safe_z_value
    
    # Phase 1: Fast approach (5mm steps)
    fast_step_size = 5.0
    z_current = z_start
    
    print("Phase 1: Fast approach (5mm steps)")
    while z_current >= z_limit:
        if not move_to_position(x, y, z_current):
            print("Failed to move to position for contact detection")
            return None
            
        _, _, fz = mean_force_data()
        print(f"Z={z_current:.3f}, Fz={fz:.4f}")
        
        # If we detect any force, switch to precise mode
        if fz <= contact_threshold:
            print(f"Force detected at Z={z_current:.3f}, switching to precise mode")
            break
        
        z_current -= fast_step_size
        time.sleep(0.1)
    
    # Phase 2: Precise contact detection (0.05mm steps)
    print("Phase 2: Precise contact detection (0.05mm steps)")
    precise_step_size = 0.05
    
    # Start from slightly above where we detected force
    z_current += fast_step_size  # Go back up one step
    
    while z_current >= z_limit:
        if not move_to_position(x, y, z_current):
            print("Failed to move to position for precise contact detection")
            return None
            
        _, _, fz = mean_force_data()
        print(f"Z={z_current:.3f}, Fz={fz:.4f}")
        
        if fz <= contact_threshold:
            global contact_threshold_z
            contact_threshold_z = z_current
            print(f"\n✅ Contact detected at Z={z_current:.3f} mm with Fz={fz:.4f}")
            return z_current
        
        z_current -= precise_step_size
        time.sleep(0.1)
    
    print("⚠️ Contact not detected within safe Z limit.")
    return None

def find_contact_point_precise(x, y, start_z=-10.0, z_limit=-30, contact_threshold=-0.03, step_size=0.1):
    """Find contact point with precise 0.1mm steps starting from a close position"""
    print(f"\nPrecise contact detection at X={x}, Y={y}")
    print(f"Starting from Z={start_z:.3f}mm, moving down in {step_size}mm steps")
    
    z_current = start_z
    
    while z_current >= z_limit:
        if not move_to_position(x, y, z_current):
            print("Failed to move to position for precise contact detection")
            return None
            
        _, _, fz = mean_force_data()
        print(f"Z={z_current:.3f}, Fz={fz:.4f}")
        
        if fz <= contact_threshold:
            global contact_threshold_z
            contact_threshold_z = z_current
            print(f"\n✅ Contact detected at Z={z_current:.3f} mm with Fz={fz:.4f}")
            return z_current
        
        z_current -= step_size
        time.sleep(DISPLACEMENT_CONFIG['step_delay'])  # Wait between steps
    
    print("⚠️ Contact not detected within safe Z limit.")
    return None

def find_contact_point_with_threshold(x, y, z_start=None, z_limit=-30, contact_threshold=-0.03):
    """Find contact point at given X,Y position with fast approach then precise detection"""
    print(f"\nFinding contact point at X={x}, Y={y}")
    print("Fast approach to sensor vicinity...")
    
    if z_start is None:
        z_start = safe_z_value
    
    # Phase 1: Fast approach (5mm steps)
    fast_step_size = 5.0
    z_current = z_start
    
    print("Phase 1: Fast approach (5mm steps)")
    while z_current >= z_limit:
        if not move_to_position(x, y, z_current):
            print("Failed to move to position for contact detection")
            return None
            
        _, _, fz = mean_force_data()
        print(f"Z={z_current:.3f}, Fz={fz:.4f}")
        
        # If we detect any force, switch to precise mode
        if fz <= contact_threshold:
            print(f"Force detected at Z={z_current:.3f}, switching to precise mode")
            break
        
        z_current -= fast_step_size
        time.sleep(0.1)
    
    # Phase 2: Precise contact detection (0.05mm steps)
    print("Phase 2: Precise contact detection (0.05mm steps)")
    precise_step_size = 0.05
    
    # Start from slightly above where we detected force
    z_current += fast_step_size  # Go back up one step
    
    while z_current >= z_limit:
        if not move_to_position(x, y, z_current):
            print("Failed to move to position for precise contact detection")
            return None
            
        _, _, fz = mean_force_data()
        print(f"Z={z_current:.3f}, Fz={fz:.4f}")
        
        if fz <= contact_threshold:
            print(f"\n✅ Contact detected at Z={z_current:.3f} mm with Fz={fz:.4f}")
            return z_current
        
        z_current -= precise_step_size
        time.sleep(0.1)
    
    print("⚠️ Contact not detected within safe Z limit.")
    return None

def perform_incremental_displacement(x, y, start_z, increment=0.1, max_displacement=2.0, images_per_step=1):
    """Perform incremental displacement with image capture at each step"""
    global current_target_pressure, current_x_angle, current_y_angle
    
    print(f"Starting incremental displacement: {increment}mm increments, max {max_displacement}mm")
    print(f"Taking {images_per_step} image at each step")
    
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        print("Warning: Camera not available. Will continue without images.")
    
    # Create timestamped directory for this Y angle session
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = f"y_displacement_{timestamp}"
    try:
        os.makedirs(session_dir, exist_ok=True)
        print(f"Created session directory: {session_dir}")
    except Exception as e:
        print(f"Error creating session directory: {e}")
        return False
    
    # Create CSV file for displacement data
    csv_filename = os.path.join(session_dir, "displacement_data.csv")
    csv_headers = ["Timestamp", "Displacement_Step", "Z_Position", "Displacement_Total", "Fx", "Fy", "Fz", "Target_Pressure", "Current_Pressure", "Image_Base_Filename"]
    
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(csv_headers)
    
    # Use current position as 0.0mm reference (no movement needed)
    print(f"Using current position as 0.0mm reference...")
    reference_z = start_z  # Current position is the reference
    print(f"Reference position: Z={reference_z:.3f}mm (this will be 0.0mm displacement)")
    
    # Wait for stabilization at current position
    print("Waiting for system to stabilize at current position...")
    time.sleep(1.0)
    
    # Take reference image at 0.0mm displacement
    print("Taking reference image at 0.0mm displacement...")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Get current pressure and calculate Z displacement
    current_pressure = read_pressure_sensor()
    current_pressure_str = f"{current_pressure:.3f}" if current_pressure is not None else "0.000"
    
    # Get target pressure
    target_pressure = current_target_pressure
    target_pressure_str = f"{target_pressure:.3f}"
    
    # Calculate Z displacement: current Z - contact threshold Z
    z_displacement = reference_z - contact_threshold_z if contact_threshold_z is not None else 0.0
    
    # Get current X and Y angles from global variables
    
    # Format: thetax_thetay_targetpressure_z_displacement_samplennumber100
    reference_image_filename = f"{current_x_angle}_{current_y_angle}_{target_pressure_str}_{z_displacement:.3f}_sample000.jpg"
    reference_image_path = os.path.join(session_dir, reference_image_filename)
    
    # Get force data at reference position
    fx, fy, fz = mean_force_data()
    print(f"Reference force data: Fx={fx:.4f}, Fy={fy:.4f}, Fz={fz:.4f}")
    
    # Capture reference image
    reference_image_captured = False
    if cap is not None:
        print(f"Capturing reference image...")
        reference_image_captured = capture_image(cap, reference_image_path)
        if reference_image_captured:
            print(f"Reference image captured: {reference_image_filename}")
        else:
            print("Failed to capture reference image")
    else:
        print("No camera available - skipping reference image capture")
    
    # Record reference data (0.0mm displacement)
    reference_data_row = [
        timestamp,
        0,  # Step 0 (reference)
        reference_z,
        0.0,  # 0.0mm displacement
        fx,
        fy,
        fz,
        target_pressure,
        current_pressure if current_pressure is not None else 0.0,
        reference_image_filename if reference_image_captured else "N/A"
    ]
    
    with open(csv_filename, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(reference_data_row)
    
    print(f"Reference data recorded (0.0mm displacement)")
    
    # Now start the actual displacement from the reference position
    current_z = reference_z
    step_count = 0
    total_displacement = 0.0
    max_steps = int(max_displacement / increment)  # Calculate total steps needed
    
    def capture_step_data(step_num, z_pos, displacement, direction="DOWN"):
        """Helper function to capture data at a specific step"""
        print(f"\n--- Step {step_num} ({direction}): Displacement = {displacement:.3f}mm ---")
        
        # Move to current Z position
        print(f"Moving to Z={z_pos:.3f}mm")
        if not move_to_position(x, y, z_pos):
            print(f"Failed to move to Z={z_pos:.3f}")
            return False
        
        # Wait for stabilization
        print("Waiting for system to stabilize...")
        time.sleep(1.0)
        
        # Get force data
        fx, fy, fz = mean_force_data()
        print(f"Force data: Fx={fx:.4f}, Fy={fy:.4f}, Fz={fz:.4f}")
        
        # Take multiple images at this displacement level
        for image_num in range(1, images_per_step + 1):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Get current pressure and calculate Z displacement
            current_pressure = read_pressure_sensor()
            current_pressure_str = f"{current_pressure:.3f}" if current_pressure is not None else "0.000"
            
            # Get target pressure
            target_pressure = current_target_pressure
            target_pressure_str = f"{target_pressure:.3f}"
            
            # Calculate Z displacement: reference Z - current Z (positive displacement as we move down)
            z_displacement = reference_z - z_pos
            
            # Format: thetax_thetay_targetpressure_z_displacement_direction_samplennumber100
            # Add direction indicator to filename
            image_filename = f"{current_x_angle}_{current_y_angle}_{target_pressure_str}_{z_displacement:.3f}_{direction}_sample{image_num:03d}.jpg"
            image_path = os.path.join(session_dir, image_filename)
            
            image_captured = False
            if cap is not None:
                print(f"Capturing image {image_num}/{images_per_step}...")
                image_captured = capture_image(cap, image_path)
                if image_captured:
                    print(f"Image captured: {image_filename}")
                else:
                    print("Failed to capture image")
            else:
                print("No camera available - skipping image capture")
            
            # Record data for each image
            data_row = [
                timestamp,
                step_num,
                z_pos,
                displacement,
                fx,
                fy,
                fz,
                target_pressure,
                current_pressure if current_pressure is not None else 0.0,
                image_filename if image_captured else "N/A"
            ]
            
            with open(csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(data_row)
            
            print(f"Data recorded for step {step_num}, image {image_num}")
        
        print(f"Completed {images_per_step} images for step {step_num}")
        return True
    
    try:
        # PHASE 1: Go DOWN (compression)
        print(f"\n{'='*50}")
        print("PHASE 1: COMPRESSION (Going DOWN)")
        print(f"{'='*50}")
        
        for step in range(1, max_steps + 1):
            step_count += 1
            current_z -= increment
            total_displacement += increment
            
            if not capture_step_data(step_count, current_z, total_displacement, "DOWN"):
                break
            
            # Safety check - stop if force becomes too high
            fx, fy, fz = mean_force_data()
            if fz < -5.0:  # Stop if compressive force exceeds 5N
                print(f"⚠️ High force detected (Fz={fz:.4f}N) - stopping compression")
                break
        
        # PHASE 2: Go UP (decompression)
        print(f"\n{'='*50}")
        print("PHASE 2: DECOMPRESSION (Going UP)")
        print(f"{'='*50}")
        
        for step in range(1, max_steps + 1):
            step_count += 1
            current_z += increment
            total_displacement -= increment
            
            if not capture_step_data(step_count, current_z, total_displacement, "UP"):
                break
    
    except KeyboardInterrupt:
        print("\nDisplacement interrupted by user!")
    except Exception as e:
        print(f"\nError during displacement: {e}")
    
    finally:
        # Cleanup
        if cap is not None:
            cap.release()
            print("Camera released")
        
        print(f"\nDisplacement session completed!")
        print(f"Total steps completed: {step_count}")
        print(f"Compression phase: {max_steps} steps down to {max_displacement:.3f}mm")
        print(f"Decompression phase: {max_steps} steps back up to 0.0mm")
        print(f"Total images captured: {step_count * images_per_step}")
        print(f"Data saved to: {csv_filename}")
        print(f"Images saved to: {session_dir}")
        
        # Return to safe height
        safe_z = start_z + z_safe_offset
        try:
            move_to_position(x, y, safe_z)
            print(f"Returned to safe height Z={safe_z:.3f}")
        except:
            print("Warning: Failed to return to safe height")
        
        return True

def move_to_contact_position(x, y):
    """Move to contact position using stored contact Z or find new one"""
    global contact_z_position
    
    if contact_z_position is None:
        # First time - find contact position
        print("First contact detection - finding Z position...")
        contact_z_position = find_contact_point(x, y)
        if contact_z_position is None:
            return False
        return True
    else:
        # Use stored contact position - move to safe height then down to contact
        safe_z = contact_z_position + z_safe_offset
        print(f"Moving to safe height Z={safe_z:.3f} (contact + {z_safe_offset}mm)")
        if not move_to_position(x, y, safe_z):
            return False
            
        print(f"Moving down to contact position Z={contact_z_position:.3f}")
        if not move_to_position(x, y, contact_z_position):
            return False
            
        return True

def get_cnc_position():
    """Query CNC for actual current position"""
    try:
        s_machine.reset_input_buffer()
        s_machine.write(b'?\n')
        s_machine.flush()
        
        start_time = time.time()
        while time.time() - start_time < 2:
            if s_machine.in_waiting:
                response = s_machine.readline().strip().decode('utf-8')
                if 'MPos:' in response:
                    # Parse position from response like: <Idle|MPos:-145.000,-45.000,-10.500|FS:0,0>
                    mpos_start = response.find('MPos:') + 5
                    mpos_end = response.find('|', mpos_start)
                    if mpos_end == -1:
                        mpos_end = len(response)
                    
                    pos_str = response[mpos_start:mpos_end]
                    x, y, z = map(float, pos_str.split(','))
                    return x, y, z
        return None, None, None
    except Exception as e:
        print(f"Error querying CNC position: {e}")
        return None, None, None

def move_up_2_steps():
    """Move 2 steps (0.2mm) above current Z position"""
    global current_x, current_y, current_z
    
    # Get actual CNC position to sync with tracked position
    actual_x, actual_y, actual_z = get_cnc_position()
    if actual_z is not None:
        print(f"DEBUG: Actual CNC position - X={actual_x:.3f}, Y={actual_y:.3f}, Z={actual_z:.3f}")
        print(f"DEBUG: Tracked position - X={current_x:.3f}, Y={current_y:.3f}, Z={current_z:.3f}")
        # Update tracked position to match actual
        current_x, current_y, current_z = actual_x, actual_y, actual_z
        print(f"DEBUG: Synced tracked position to actual CNC position")
    else:
        print("DEBUG: Could not query CNC position, using tracked position")
    
    # Calculate new Z position (2 steps = 2 * 0.1mm = 0.2mm up)
    step_size = 0.1  # mm per step
    new_z = current_z + (2 * step_size)
    
    print(f"DEBUG: Step size = {step_size}mm, 2 steps = {2 * step_size}mm")
    print(f"DEBUG: Current Z = {current_z:.3f}, New Z = {new_z:.3f}, Difference = {new_z - current_z:.3f}mm")
    print(f"Moving up 2 steps (0.2mm) from Z={current_z:.3f} to Z={new_z:.3f}")
    
    if move_to_position(current_x, current_y, new_z):
        print(f"Successfully moved up 2 steps to Z={new_z:.3f}")
        return True
    else:
        print("Failed to move up 2 steps")
        return False

# ========== FORCE DATA ==========
def mean_force_data():
    try:
        Fx_sum = Fy_sum = Fz_sum = 0
        valid_readings = 0
        
        for i in range(5):
            ft_data = get_FT_data()
            if ft_data is not None and len(ft_data) >= 3:
                Fx_sum += ft_data[0]
                Fy_sum += ft_data[1]
                Fz_sum += ft_data[2]
                valid_readings += 1
            time.sleep(0.1)
        
        if valid_readings > 0:
            return Fx_sum / valid_readings, Fy_sum / valid_readings, Fz_sum / valid_readings
        else:
            print("No valid force readings obtained")
            return 0, 0, 0
            
    except Exception as e:
        print(f"Error getting mean force data: {e}")
        return 0, 0, 0

# ========== INTERACTIVE CONTROL ==========
def process_x_command(user_input):
    """Process X command and return success status"""
    try:
        if user_input.startswith('X') and len(user_input) == 2:
            index = int(user_input[1])
            if 0 <= index <= 4:
                angle = thetaXAngles[index]
                print(f"X{index} = {angle}°")
                
                # Send command to Arduino
                if send_motor_command(user_input):
                    print(f"Motor moved to X{index} ({angle}°)")
                    
                    # Calculate new X position for CNC
                    new_x = calculate_x_from_theta(angle)
                    print(f"Calculated CNC X position: {new_x:.3f}")
                    global current_x, current_x_angle
                    current_x = new_x
                    current_x_angle = angle
                    
                    # Move CNC to new X position at safe height (close to sensor)
                    # change z value for cnc after calculation of x position
                    approach_z = -9.0  # Adjusted for new initial Z position
                    print(f"Moving to approach position Z={approach_z:.3f}mm")
                    if not move_to_position(new_x, y_cord, approach_z):
                        print("Failed to move CNC to approach position")
                        return False
                    
                    # Find contact point with 0.1mm steps
                    print("Starting precise contact detection (0.1mm steps)...")
                    contact_z = find_contact_point_precise(new_x, y_cord, start_z=approach_z)
                    if contact_z is not None:
                        print(f"Contact found at Z={contact_z:.3f}")
                        # Move up 0.1mm from contact
                        safe_contact_z = contact_z + 0.1
                        print(f"Moving up 0.1mm to Z={safe_contact_z:.3f}")
                        if not move_to_position(new_x, y_cord, safe_contact_z):
                            print("Failed to move to safe contact position")
                            return False
                        
                        # Store the contact position for future use
                        global last_contact_z
                        last_contact_z = safe_contact_z
                        
                        print(f"\n✅ X{index} ({angle}°) set successfully!")
                        print(f"Contact established at Z={safe_contact_z:.3f}mm")
                        print("Returning to main menu. You can now set Y angle, pressure, or start displacement.")
                        return True
                    else:
                        print("Contact not found at new X position!")
                        return False
                else:
                    print("Failed to send motor command")
                    return False
            else:
                print("Invalid X index. Use X0-X4")
                return False
        else:
            print("Invalid X command format. Use X0, X1, X2, X3, or X4")
            return False
    except ValueError:
        print("Invalid X command format. Use X0, X1, X2, X3, or X4")
        return False
    except Exception as e:
        print(f"Error processing X command: {e}")
        return False

def process_y_command(user_input):
    """Process Y command and return success status"""
    try:
        if user_input.startswith('Y') and len(user_input) == 2:
            index = int(user_input[1])
            if 0 <= index <= 6:
                angle = thetaYAngles[index]
                print(f"Y{index} = {angle}°")
                
                # Send command to Arduino
                if send_motor_command(user_input):
                    print(f"Motor moved to Y{index} ({angle}°)")
                    print(f"CNC Y position remains at: {y_cord}")
                    
                    # Update global Y angle
                    global current_y_angle
                    current_y_angle = angle
                    
                    # Y command completed successfully - no pressure prompt needed
                    print(f"\nY{index} ({angle}°) set successfully.")
                    print("Use 'D' or 'DISP' command to start incremental displacement and image capture.")
                    return True
                else:
                    print("Failed to send motor command")
                    return False
            else:
                print("Invalid Y index. Use Y0-Y6")
                return False
        else:
            print("Invalid Y command format. Use Y0, Y1, Y2, Y3, Y4, Y5, or Y6")
            return False
    except ValueError:
        print("Invalid Y command format. Use Y0, Y1, Y2, Y3, Y4, Y5, or Y6")
        return False
    except Exception as e:
        print(f"Error processing Y command: {e}")
        return False

def process_pressure_command(user_input):
    """Process pressure command and return success status"""
    try:
        if user_input.startswith('M') and len(user_input) > 1:
            try:
                value = int(user_input[1:])
                if send_pressure_command(user_input):
                    print(f"Pressure set to {value}")
                    return True
                else:
                    print("Failed to send pressure command")
                    return False
            except ValueError:
                print("Invalid M command. Use format M<number>")
                return False
        elif user_input in ['R', 'Z']:
            if send_pressure_command(user_input):
                print(f"Pressure command {user_input} executed")
                return True
            else:
                print("Failed to send pressure command")
                return False
        else:
            print("Invalid pressure command. Use M<number>, R, or Z")
            return False
    except Exception as e:
        print(f"Error processing pressure command: {e}")
        return False

# ========== IMAGE CAPTURE FUNCTIONS ==========
def initialize_camera():
    if not CV2_AVAILABLE:
        print("OpenCV not available - camera disabled")
        return None

    def try_open(idx, backend):
        cap = cv2.VideoCapture(idx, backend) if backend is not None else cv2.VideoCapture(idx)
        if not cap.isOpened():
            return None
        # basic config
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass
        # Enable AUTO exposure on Windows backends (0.75 quirk)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        # Warm up
        for _ in range(15):
            cap.read()
        return cap

    # Order: your requested index with DSHOW → MSMF → ANY, then sweep 0..5
    backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
    indices  = [camera_index] + [i for i in range(0,6) if i != camera_index]

    for b in backends:
        # Try the preferred index first
        cap = try_open(camera_index, b)
        if cap:
            print(f"Camera initialized at index {camera_index} with backend {b}")
            return cap
        # If that fails, sweep other likely indices
        for idx in indices:
            cap = try_open(idx, b)
            if cap:
                print(f"Camera initialized at index {idx} with backend {b}")
                return cap

    print(f"Error: Could not open any camera with DSHOW/MSMF/ANY")
    return None

def capture_image(cap, filename):
    """Capture and save an image"""
    if not CV2_AVAILABLE:
        print("OpenCV not available - cannot capture image")
        return False
    
    try:
        if cap is None:
            print("Camera not available")
            return False
        
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture image")
            return False
        
        # Save image
        cv2.imwrite(filename, frame)
        print(f"Image saved: {filename}")
        return True
        
    except Exception as e:
        print(f"Error capturing image: {e}")
        return False

def create_experiment_directory():
    """Create directory for experiment images with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    experiment_dir = f"experiment_{timestamp}"
    
    try:
        os.makedirs(experiment_dir, exist_ok=True)
        print(f"Created experiment directory: {experiment_dir}")
        return experiment_dir
    except Exception as e:
        print(f"Error creating experiment directory: {e}")
        return None

# ========== AUTOMATED EXPERIMENT FUNCTION ==========
def run_automated_experiment():
    """Run automated experiment through all pressure values, X angles, and Y angles"""
    global contact_z_position
    
    print("\n" + "="*60)
    print("STARTING AUTOMATED EXPERIMENT")
    print("="*60)
    print(f"Pressure values: {pressure_values}")
    print(f"X angles: {thetaXAngles}")
    print(f"Y angles: {thetaYAngles}")
    print("="*60)
    
    # Initialize camera
    print("\nInitializing camera...")
    cap = initialize_camera()
    if cap is None:
        print("Warning: Camera not available. Experiment will continue without images.")
    
    # Create experiment directory
    experiment_dir = create_experiment_directory()
    if experiment_dir is None:
        print("Error: Could not create experiment directory. Exiting.")
        return False
    
    # Initialize systems if not already done
    if contact_z_position is None:
        print("\nInitializing systems...")
        if not initialize_cnc():
            print("Failed to initialize CNC. Exiting.")
            return False
        
        initialize_ATI_mini40()
        initialize_pressure_control()
        initialize_motor_control()
        
        # Find initial contact position
        print(f"\nFinding initial contact position at X={x_cord}, Y={y_cord}")
        if not move_to_position(x_cord, y_cord, safe_z_value):
            print("Failed to reach initial position. Exiting.")
            return False
        
        contact_z_position = find_contact_point(x_cord, y_cord)
        if contact_z_position is None:
            print("Failed to find contact position. Exiting.")
            return False
        
        print(f"Contact position found: Z={contact_z_position:.3f}mm")
    
    # Create CSV file for experiment data
    csv_filename = os.path.join(experiment_dir, "experiment_data.csv")
    csv_headers = ["Timestamp", "Target_Pressure", "Current_Pressure", "X_Index", "X_Angle", "Y_Index", "Y_Angle", 
                   "CNC_X", "CNC_Y", "CNC_Z", "Fx", "Fy", "Fz", "Image_Filename"]
    
    with open(csv_filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(csv_headers)
    
    total_combinations = len(pressure_values) * len(thetaXAngles) * len(thetaYAngles)
    current_combination = 0
    
    try:
        # Main experiment loop
        for pressure_idx, pressure in enumerate(pressure_values):
            print(f"\n{'='*40}")
            print(f"PRESSURE SETTING: M{pressure} ({pressure_idx + 1}/{len(pressure_values)})")
            print(f"{'='*40}")
            
            # Set pressure
            if not send_pressure_command(f"M{pressure}"):
                print(f"Failed to set pressure to M{pressure}. Skipping this pressure setting.")
                continue
            
            # Wait for pressure to stabilize
            print(f"Waiting for pressure M{pressure} to stabilize...")
            time.sleep(3)
            
            # Return to zero positions after pressure change
            if not return_to_zero_positions():
                print("Failed to return to zero positions. Skipping this pressure setting.")
                continue
            
            for x_idx, x_angle in enumerate(thetaXAngles):
                print(f"\n--- X ANGLE: {x_angle}° (X{x_idx}) ---")
                
                # Send X command to Arduino
                if not send_motor_command(f"X{x_idx}"):
                    print(f"Failed to set X{x_idx}. Skipping this X angle.")
                    continue
                
                # Calculate and move to new X position
                new_x = calculate_x_from_theta(x_angle)
                safe_z = contact_z_position + z_safe_offset
                
                if not move_to_position(new_x, y_cord, safe_z):
                    print(f"Failed to move to X position {new_x}. Skipping this X angle.")
                    continue
                
                for y_idx, y_angle in enumerate(thetaYAngles):
                    current_combination += 1
                    print(f"\n  Y ANGLE: {y_angle}° (Y{y_idx}) - Progress: {current_combination}/{total_combinations}")
                    
                    # Send Y command to Arduino
                    if not send_motor_command(f"Y{y_idx}"):
                        print(f"  Failed to set Y{y_idx}. Skipping this Y angle.")
                        continue
                    
                    # Move down to contact position
                    if not move_to_position(new_x, y_cord, contact_z_position):
                        print(f"  Failed to move to contact position. Skipping this Y angle.")
                        continue
                    
                    # Wait for system to stabilize
                    print(f"  Waiting for system to stabilize...")
                    time.sleep(image_delay)
                    
                    # Get force data
                    fx, fy, fz = mean_force_data()
                    print(f"  Force data: Fx={fx:.4f}, Fy={fy:.4f}, Fz={fz:.4f}")
                    
                    # Capture image
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    
                    # Get current pressure and calculate Z displacement
                    current_pressure = read_pressure_sensor()
                    current_pressure_str = f"{current_pressure:.3f}" if current_pressure is not None else "0.000"
                    
                    # Get target pressure from the current pressure setting (convert index to actual pressure)
                    target_pressure = pressure_mapping.get(pressure, pressure)  # Use mapping or fallback to index
                    target_pressure_str = f"{target_pressure:.3f}"
                    
                    # Calculate Z displacement: current Z - contact threshold Z
                    z_displacement = contact_z_position - contact_threshold_z if contact_threshold_z is not None else 0.0
                    
                    # Format: thetax_thetay_targetpressure_z_displacement_samplennumber100
                    image_filename = f"{x_angle}_{y_angle}_{target_pressure_str}_{z_displacement:.3f}_sample{current_combination:03d}.jpg"
                    image_path = os.path.join(experiment_dir, image_filename)
                    
                    image_captured = False
                    if cap is not None:
                        image_captured = capture_image(cap, image_path)
                        if not image_captured:
                            print(f"  Warning: Failed to capture image")
                    else:
                        print(f"  No camera available - skipping image capture")
                    
                    # Record data
                    data_row = [
                        timestamp,
                        target_pressure,
                        current_pressure if current_pressure is not None else 0.0,
                        x_idx,
                        x_angle,
                        y_idx,
                        y_angle,
                        new_x,
                        y_cord,
                        contact_z_position,
                        fx,
                        fy,
                        fz,
                        image_filename if image_captured else "N/A"
                    ]
                    
                    with open(csv_filename, 'a', newline='') as csvfile:
                        writer = csv.writer(csvfile)
                        writer.writerow(data_row)
                    
                    print(f"  Data recorded: {image_filename}")
                    
                    # Move back to safe height for next iteration
                    if not move_to_position(new_x, y_cord, safe_z):
                        print(f"  Warning: Failed to move to safe height")
    
    except KeyboardInterrupt:
        print("\n\nExperiment interrupted by user!")
        print("Saving current progress...")
    except Exception as e:
        print(f"\n\nError during experiment: {e}")
        print("Saving current progress...")
    
    finally:
        # Cleanup
        if cap is not None:
            cap.release()
            print("Camera released")
        
        print(f"\nExperiment completed!")
        print(f"Data saved to: {csv_filename}")
        print(f"Images saved to: {experiment_dir}")
        print(f"Total combinations completed: {current_combination}/{total_combinations}")
        
        # Return to safe position
        try:
            return_to_zero_positions()
        except:
            pass
        
        return True

# ========== MAIN ==========
def main():
    print("Starting Interactive CNC Control Program...")
    
    try:
        # Initialize systems
        if not initialize_cnc():
            print("Failed to initialize CNC. Exiting.")
            return
        
        initialize_ATI_mini40()
        initialize_pressure_control()
        initialize_motor_control()
        
        # Move to initial position
        print(f"\nMoving to initial position: X={x_cord}, Y={y_cord}, Z={safe_z_value}")
        if not move_to_position(x_cord, y_cord, safe_z_value):
            print("Failed to reach initial position. Exiting.")
            return
        
        # Using fixed Z position from settings
        print(f"\nUsing fixed Z position: {z_cord:.3f}mm")
        
        # Move to fixed Z position
        if not move_to_position(x_cord, y_cord, z_cord):
            print("Failed to move to Z position. Exiting.")
            return
        
        print(f"Initialized at Z={z_cord:.3f}mm")
        print("Pressure will be prompted when using X or Y commands.")
        
        # Main continuous command loop
        print("\n" + "="*50)
        print("CONTINUOUS COMMAND MODE")
        print("="*50)
        print("Available commands:")
        print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
        print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
        print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
        print("  R - Reset pressure")
        print("  Z - Zero pressure")
        print("  UP - Move up 2 steps (0.2mm) from current Z position")
        print("  auto - Run automated experiment (M0, M1, M2, M3, M4 through all X/Y angles)")
        print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
        print("  exit - Exit program")
        
        while True:
            try:
                user_input = input("\nEnter command: ").strip().upper()
                
                if user_input == 'EXIT':
                    print("Exiting...")
                    break
                
                # Process automated experiment
                elif user_input == 'AUTO':
                    print("\nStarting automated experiment...")
                    if run_automated_experiment():
                        print("Automated experiment completed successfully!")
                    else:
                        print("Automated experiment failed or was interrupted.")
                    print("\nReturning to command mode...")
                    print("\nAvailable commands:")
                    print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
                    print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
                    print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
                    print("  R - Reset pressure")
                    print("  Z - Zero pressure")
                    print("  UP - Move up 2 steps (0.2mm) from current Z position")
                    print("  auto - Run automated experiment")
                    print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
                    print("  exit - Exit program")
                
                # Process X command
                elif user_input.startswith('X'):
                    if process_x_command(user_input):
                        print("\nX command completed successfully.")
                        print("\nAvailable commands:")
                        print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
                        print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
                        print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
                        print("  R - Reset pressure")
                        print("  Z - Zero pressure")
                        print("  UP - Move up 2 steps (0.2mm) from current Z position")
                        print("  auto - Run automated experiment")
                        print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
                        print("  exit - Exit program")
                    else:
                        print("X command failed. Try again.")
                
                # Process Y command
                elif user_input.startswith('Y'):
                    if process_y_command(user_input):
                        print("\nY command completed successfully.")
                        print("\nAvailable commands:")
                        print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
                        print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
                        print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
                        print("  R - Reset pressure")
                        print("  Z - Zero pressure")
                        print("  UP - Move up 2 steps (0.2mm) from current Z position")
                        print("  auto - Run automated experiment")
                        print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
                        print("  exit - Exit program")
                    else:
                        print("Y command failed. Try again.")
                
                # Process pressure command
                elif user_input.startswith('M') or user_input in ['R', 'Z']:
                    if process_pressure_command(user_input):
                        print("Pressure changed. Returning to zero positions...")
                        if return_to_zero_positions():
                            print("Pressure command completed successfully.")
                            print("\nAvailable commands:")
                            print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
                            print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
                            print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
                            print("  R - Reset pressure")
                            print("  Z - Zero pressure")
                            print("  UP - Move up 2 steps (0.2mm) from current Z position")
                            print("  auto - Run automated experiment")
                            print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
                            print("  exit - Exit program")
                        else:
                            print("Failed to return to zero positions")
                    else:
                        print("Invalid pressure command. Try again (M0-M4, R, Z):")
                
                # Process manual displacement/image capture command
                elif user_input in ['D', 'DISP']:
                    global current_x, last_contact_z
                    if last_contact_z is None:
                        print("Error: No contact Z position found. Please set X and Y first.")
                    else:
                        print("Starting manual incremental displacement and image capture...")
                        images_per_step = DISPLACEMENT_CONFIG['images_per_step']
                        increment = DISPLACEMENT_CONFIG['increment']
                        max_displacement = DISPLACEMENT_CONFIG['max_displacement']
                        success = perform_incremental_displacement(
                            current_x, y_cord, last_contact_z,
                            increment=increment,
                            max_displacement=max_displacement,
                            images_per_step=images_per_step
                        )
                        if success:
                            print("Manual displacement and image capture completed.")
                        else:
                            print("Manual displacement failed.")
                
                # Process move up command
                elif user_input == 'UP':
                    if move_up_2_steps():
                        print("Move up command completed successfully.")
                    else:
                        print("Move up command failed.")
                
                else:
                    print("Invalid command. Available commands:")
                    print("  X0-X4 - Set X angle (0°, 5.625°, 11.25°, 16.875°, 22.5°)")
                    print("  Y0-Y6 - Set Y angle (0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°)")
                    print("  M0-M4 - Set pressure (0=0psi, 1=0.29psi, 2=0.58psi, 3=0.87psi, 4=1.16psi) - can be changed anytime")
                    print("  R - Reset pressure")
                    print("  Z - Zero pressure")
                    print("  UP - Move up 2 steps (0.2mm) from current Z position")
                    print("  auto - Run automated experiment")
                    print("  D or DISP - Start incremental displacement and image capture (after Y is set)")
                    print("  exit - Exit program")
                    
            except KeyboardInterrupt:
                print("\nProgram interrupted by user")
                break
            except Exception as e:
                print(f"Error processing command: {e}")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        try:
            s_machine.close()
            if 'pressure_ser' in globals():
                pressure_ser.close()
            if 'motor_ser' in globals():
                motor_ser.close()
        except:
            pass

if __name__ == "__main__":
    main()