#!/usr/bin/env python3
"""
Test script for the automated experiment function.
This script demonstrates how to run the automated experiment that goes through:
- Pressure values: M0, M5, M10
- X angles: 0°, 5.625°, 11.25°, 16.875°, 22.5°
- Y angles: 0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°

At each combination, it will:
1. Set the pressure
2. Move to the X angle
3. Move to the Y angle
4. Take an image
5. Record force data
6. Save everything to a timestamped directory
"""

import sys
import os

# Add the current directory to the path so we can import from fullscript
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from fullscript import run_automated_experiment, initialize_cnc, initialize_ATI_mini40, initialize_pressure_control, initialize_motor_control
    
    def main():
        print("Automated Experiment Test Script")
        print("="*50)
        print("This script will run the automated experiment through all combinations:")
        print("- Pressure: M0, M5, M10")
        print("- X angles: 0°, 5.625°, 11.25°, 16.875°, 22.5°")
        print("- Y angles: 0°, 5.625°, 11.25°, 16.875°, 22.5°, 28.125°, 33.75°")
        print()
        print("Total combinations: 3 × 5 × 7 = 105")
        print()
        
        # Ask for confirmation
        response = input("Do you want to proceed with the automated experiment? (yes/no): ").strip().lower()
        if response not in ['yes', 'y']:
            print("Experiment cancelled.")
            return
        
        print("\nStarting automated experiment...")
        print("Note: You can interrupt at any time with Ctrl+C")
        print()
        
        # Run the automated experiment
        success = run_automated_experiment()
        
        if success:
            print("\n✅ Automated experiment completed successfully!")
        else:
            print("\n❌ Automated experiment failed or was interrupted.")
    
    if __name__ == "__main__":
        main()

except ImportError as e:
    print(f"Error importing from fullscript.py: {e}")
    print("Make sure fullscript.py is in the same directory as this test script.")
except Exception as e:
    print(f"Unexpected error: {e}") 