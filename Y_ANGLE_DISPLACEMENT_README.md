# Y Angle Displacement Functionality

## Overview

The Y angle command has been modified to implement a new displacement protocol that includes:
1. **Force threshold contact detection**
2. **0.1mm gap verification**
3. **Incremental displacement with image capture**
4. **1 image per 0.1mm step until 2mm total displacement**

## New Y Command Behavior

### When you input a Y angle (Y0-Y6):

1. **Motor Movement**
   - Arduino Y motor moves to specified angle
   - CNC Y position remains fixed at -44mm

2. **Contact Detection**
   - System uses smart two-phase approach:
     - **Phase 1**: Fast approach (5mm steps) until force detected
     - **Phase 2**: Precise detection (0.05mm steps) for accurate contact
   - Uses `find_contact_point_with_threshold()` function
   - Detects contact when Fz ≤ -0.03N (configurable)

3. **Gap Positioning**
   - Backtracks 0.1mm above contact position
   - Verifies gap by checking force reading
   - Warns if force > 0.01N (gap may be too small)

4. **Incremental Displacement**
   - Starts at 0.1mm above contact
   - Moves down in 0.1mm increments
   - Takes 1 image at each step
   - Continues until 2mm total displacement or force limit

## Function Details

### `find_contact_point_with_threshold(x, y, ...)`
- **Purpose**: Find contact position using smart two-phase detection
- **Parameters**:
  - `contact_threshold = -0.03N` (default)
  - `z_limit = -10mm` (safety limit)
- **Two-Phase Approach**:
  - **Phase 1**: Fast approach (5mm steps) until force detected
  - **Phase 2**: Precise detection (0.05mm steps) for accurate contact
- **Returns**: Z position where contact is detected

### `perform_incremental_displacement(x, y, start_z, ...)`
- **Purpose**: Perform incremental displacement with image capture
- **Parameters**:
  - `increment = 0.1mm` (step size)
  - `max_displacement = 2.0mm` (total displacement)
  - `images_per_step = 1` (image per position)
- **Safety**: Stops if Fz < -5.0N (excessive force)

## Data Collection

### Directory Structure
```
y_displacement_YYYYMMDD_HHMMSS/
├── displacement_data.csv          # Force and position data
├── step001_disp0.000mm_YYYYMMDD_HHMMSS.jpg
├── step002_disp0.100mm_YYYYMMDD_HHMMSS.jpg
├── step003_disp0.200mm_YYYYMMDD_HHMMSS.jpg
├── ...
└── step020_disp1.900mm_YYYYMMDD_HHMMSS.jpg
```

### CSV Data Format
```csv
Timestamp,Displacement_Step,Z_Position,Displacement_Total,Fx,Fy,Fz,Image_Filename
20241201_143025,1,-8.120,0.000,0.1234,0.5678,-0.9012,step001_disp0.000mm_20241201_143025.jpg
20241201_143030,2,-8.220,0.100,0.2345,0.6789,-1.0123,step002_disp0.100mm_20241201_143030.jpg
...
```

## Usage

### Interactive Mode
```bash
python fullscript.py
# Set X angle first (X0-X4)
# Then input Y angle (Y0-Y6)
# System will automatically:
# 1. Find contact position
# 2. Verify 0.1mm gap
# 3. Perform incremental displacement
# 4. Capture 100 images per step
# 5. Record all data to CSV
```

### Example Session
```
Enter command: X2
X2 = 11.25°
Motor moved to X2 (11.25°)
Calculated CNC X position: -145.234
First X command - will find and store contact Z position
Contact Z position stored: -8.220mm

Enter command: Y3
Y3 = 16.875°
Motor moved to Y3 (16.875°)
CNC Y position remains at: -44
Finding contact position with force threshold detection...
Fast approach to sensor vicinity...
Phase 1: Fast approach (5mm steps)
Z=-8.000, Fz=0.0000
Z=-13.000, Fz=0.0000
Z=-18.000, Fz=-0.0123
Force detected at Z=-18.000, switching to precise mode
Phase 2: Precise contact detection (0.05mm steps)
Z=-10.000, Fz=0.0000
Z=-10.050, Fz=0.0000
Z=-10.100, Fz=-0.0123
Z=-10.150, Fz=-0.0456
✅ Contact detected at Z=-10.150 mm with Fz=-0.0456
Moving to safe contact position Z=-8.050 (contact + 0.1mm)
Gap verification - Fz=0.0023N
Starting incremental displacement with image capture...
Starting incremental displacement: 0.1mm increments, max 2.0mm
Taking 1 image at each step
Created session directory: y_displacement_20241201_143025

--- Step 1: Displacement = 0.000mm ---
Moving to Z=-8.050mm
Waiting for system to stabilize...
Force data: Fx=0.1234, Fy=0.5678, Fz=0.0023
Capturing image...
Image captured: step001_disp0.000mm_20241201_143025.jpg
Data recorded for step 1

--- Step 2: Displacement = 0.100mm ---
Moving to Z=-8.150mm
...
```

## Configuration

### Force Thresholds
```python
# In fullscript.py
contact_threshold = -0.03  # Contact detection threshold (N)
force_limit = -5.0        # Maximum safe force (N)
gap_verification = 0.01   # Gap verification threshold (N)
```

### Displacement Parameters
```python
# In perform_incremental_displacement()
increment = 0.1           # Step size (mm)
max_displacement = 2.0    # Total displacement (mm)
images_per_step = 1       # Image per position
```

### Timing
```python
stabilization_delay = 1.0  # Seconds to wait after movement
image_delay = 0.05        # Seconds between images
```

## Safety Features

1. **Force Monitoring**: Stops if compressive force exceeds 5N
2. **Gap Verification**: Warns if 0.1mm gap is not maintained
3. **Safe Return**: Always returns to safe height after completion
4. **Interruption Handling**: Gracefully handles Ctrl+C interruption
5. **Error Recovery**: Continues operation even if individual images fail

## Expected Output

### Total Data Volume
- **20 displacement steps** (0.0mm to 1.9mm in 0.1mm increments)
- **20 images** (1 image × 20 steps)
- **20 CSV rows** (one per displacement step)
- **~30-60 seconds** per Y angle (depending on camera speed)

### File Naming Convention
- **Images**: `step{step:03d}_disp{displacement:.3f}mm_{timestamp}.jpg`
- **Directory**: `y_displacement_{timestamp}/`
- **CSV**: `displacement_data.csv`

This modification provides comprehensive data collection for analyzing the sensor's response to incremental displacement at different angular orientations. 