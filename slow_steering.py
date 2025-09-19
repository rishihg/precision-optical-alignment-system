import sys
import time
from optical_aligner import OpticalAligner

# --- CONFIGURATION ---
# UPDATE THESE TO MATCH YOUR SYSTEM'S DEVICE PATHS/SYMLINKS
KQD_PORT = "/dev/kqd"      # Symlink for the Quadrant Detector
KM_PORT = "/dev/kinesis"   # Symlink for the Kinesis Motor (Goniometer)
# The Tic controller is found automatically and does not need a port.
# ---------------------

def main():
    """
    Main function to initialize, calibrate, and run the slow optical aligner.
    """
    print("--- Starting Slow Steering Routine ---")
    # Initialize the aligner with all ports needed for slow steering
    aligner = OpticalAligner(kqd_port=KQD_PORT, km_port=KM_PORT)

    try:
        # Connect to all slow steering hardware
        aligner.connect_slow_steering()

        # Run the calibration routine to build the control matrix
        # This is essential for the software PID loop to work correctly.
        aligner.calibrate()

        # Start the active slow feedback loop
        aligner.run_slow_feedback_loop()

    except Exception as e:
        print(f"\nA critical error occurred: {e}")
        sys.exit(1)
    finally:
        # This block ensures hardware is always disconnected safely
        print("\nApplication shutting down.")
        aligner.disconnect_slow_steering()

if __name__ == "__main__":
    main()

