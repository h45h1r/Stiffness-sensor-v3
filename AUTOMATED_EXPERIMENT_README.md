# Automated Experiment Function

## Overview

The automated experiment function (`run_automated_experiment()`) automatically runs through all combinations of pressure values, X angles, and Y angles, taking images and recording force data at each position.

## Experiment Parameters

### Pressure Values
- **M0**: Pressure setting 0
- **M5**: Pressure setting 5  
- **M10**: Pressure setting 10

### X Angles (5 positions)
- **X0**: 0°
- **X1**: 5.625°
- **X2**: 11.25°
- **X3**: 16.875°
- **X4**: 22.5°

### Y Angles (7 positions)
- **Y0**: 0°
- **Y1**: 5.625°
- **Y2**: 11.25°
- **Y3**: 16.875°
- **Y4**: 22.5°
- **Y5**: 28.125°
- **Y6**: 33.75°

## Total Combinations
- **3 pressure values** × **5 X angles** × **7 Y angles** = **105 total combinations**

## How It Works

### 1. Initialization
- Initializes camera (if available)
- Creates timestamped experiment directory
- Initializes CNC, sensor, and Arduino connections
- Finds initial contact position

### 2. Experiment Loop
For each pressure value (M0 → M5 → M10):
1. **Set pressure** using Arduino
2. **Wait for stabilization** (3 seconds)
3. **Return to zero positions**

For each X angle (X0 → X1 → X2 → X3 → X4):
1. **Send X command** to Arduino motor
2. **Calculate CNC X position** based on angle
3. **Move CNC to X position** at safe height

For each Y angle (Y0 → Y1 → Y2 → Y3 → Y4 → Y5 → Y6):
1. **Send Y command** to Arduino motor
2. **Move down to contact position**
3. **Wait for stabilization** (2 seconds)
4. **Capture force data** (Fx, Fy, Fz)
5. **Take image** (if camera available)
6. **Record all data** to CSV file
7. **Move back to safe height**

### 3. Data Collection
Each combination records:
- **Timestamp**
- **Pressure setting**
- **X angle index and value**
- **Y angle index and value**
- **CNC coordinates** (X, Y, Z)
- **Force data** (Fx, Fy, Fz)
- **Image filename** (if captured)

## Usage

### Method 1: Interactive Mode
1. Run `python fullscript.py`
2. Initialize systems when prompted
3. Type `auto` and press Enter
4. The experiment will start automatically

### Method 2: Direct Function Call
```python
from fullscript import run_automated_experiment

# Run the experiment
success = run_automated_experiment()
```

### Method 3: Test Script
```bash
python test_automated_experiment.py
```

## Output Structure

### Directory Structure
```
experiment_YYYYMMDD_HHMMSS/
├── experiment_data.csv          # All experiment data
├── P0_X0_0deg_Y0_0deg_YYYYMMDD_HHMMSS.jpg
├── P0_X0_0deg_Y1_5.625deg_YYYYMMDD_HHMMSS.jpg
├── P0_X0_0deg_Y2_11.25deg_YYYYMMDD_HHMMSS.jpg
├── ...
├── P10_X4_22.5deg_Y6_33.75deg_YYYYMMDD_HHMMSS.jpg
└── (105 total images)
```

### CSV Data Format
```csv
Timestamp,Pressure,X_Index,X_Angle,Y_Index,Y_Angle,CNC_X,CNC_Y,CNC_Z,Fx,Fy,Fz,Image_Filename
20241201_143022,0,0,0.0,0,0.0,-146.000,-44.000,-8.220,0.1234,0.5678,-0.9012,P0_X0_0deg_Y0_0deg_20241201_143022.jpg
20241201_143025,0,0,0.0,1,5.625,-146.000,-44.000,-8.220,0.2345,0.6789,-0.0123,P0_X0_0deg_Y1_5.625deg_20241201_143025.jpg
...
```

## Configuration

### Camera Settings
```python
# In fullscript.py
camera_index = 0  # Camera device index
image_delay = 2.0  # Delay before taking image (seconds)
```

### Experiment Parameters
```python
# In fullscript.py
pressure_values = [0, 5, 10]  # Pressure settings to test
```

## Requirements

### Hardware
- CNC machine connected to COM5
- Arduino for motor control (X/Y) connected to COM3
- Arduino for pressure control (M/R/Z) connected to COM4
- ATI Mini40 force sensor
- Camera (optional - experiment works without camera)

### Software Dependencies
- `pyserial` - Serial communication
- `numpy` - Numerical operations
- `nidaqmx` - DAQ data acquisition
- `opencv-python` - Image capture (optional)
- `csv` - Data logging
- `datetime` - Timestamping

## Installation

```bash
# Install required packages
pip install pyserial numpy nidaqmx opencv-python

# Or install without OpenCV (images will be disabled)
pip install pyserial numpy nidaqmx
```

## Safety Features

### Error Handling
- **Graceful failures**: If one combination fails, continues with next
- **Progress tracking**: Shows current progress (X/105 combinations)
- **Data preservation**: Saves data even if experiment is interrupted
- **Safe positioning**: Always returns to safe height between measurements

### Interruption
- **Ctrl+C**: Safely interrupts experiment and saves current progress
- **Emergency stop**: Can be added to CNC emergency stop button

## Troubleshooting

### Common Issues

1. **Camera not found**
   - Check camera connection
   - Verify camera_index setting
   - Experiment continues without images

2. **Serial port errors**
   - Check COM port settings
   - Verify Arduino connections
   - Restart Arduino devices

3. **CNC movement errors**
   - Check CNC connection
   - Verify coordinate limits
   - Check for obstructions

4. **Force sensor errors**
   - Check DAQ connection
   - Verify calibration file
   - Check sensor bias

### Debug Mode
Add debug prints by modifying the functions in `fullscript.py` to see detailed information about each step.

## Customization

### Adding More Pressure Values
```python
pressure_values = [0, 2, 5, 7, 10]  # Add more values
```

### Changing Angle Ranges
```python
thetaXAngles = [0, 10, 20, 30]  # Custom X angles
thetaYAngles = [0, 15, 30, 45]  # Custom Y angles
```

### Modifying Delays
```python
image_delay = 5.0  # Longer stabilization time
```

## Support

For issues or questions about the automated experiment function, check:
1. Serial port connections
2. Hardware setup
3. Software dependencies
4. Error messages in console output 