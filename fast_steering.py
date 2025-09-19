import sys
from optical_aligner import OpticalAligner

# --- CONFIGURATION ---
# UPDATE THIS TO MATCH YOUR SYSTEM'S DEVICE PATH / SERIAL NUMBER
KQD_PORT = "/dev/kqd"     # Symlink for the Quadrant Detector/Piezo Controller
# ---------------------

def main():
    """
    Main function to initialize and run the interactive fast steering controller.
    """
    print("--- Starting Fast Steering Routine ---")
    # Initialize the aligner with only the KQD port
    aligner = OpticalAligner(kqd_port=KQD_PORT)

    try:
        # Use the specific connection method for fast steering hardware
        aligner.connect_fast_steering()

        # Call the main interactive method, which contains its own menu loop
        aligner.manage_fast_steering_mode()

    except Exception as e:
        print(f"\nA critical error occurred: {e}")
        sys.exit(1)
    finally:
        # Use the specific disconnection method for fast steering hardware
        print("\nApplication shutting down.")
        aligner.disconnect_fast_steering()

if __name__ == "__main__":
    main()

