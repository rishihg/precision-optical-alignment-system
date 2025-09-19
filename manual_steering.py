import sys
from optical_aligner import OpticalAligner

# --- CONFIGURATION ---
# UPDATE THESE TO MATCH YOUR SYSTEM'S DEVICE PATHS/SYMLINKS
KQD_PORT = "/dev/kqd"      # Symlink for the Quadrant Detector
KM_PORT = "/dev/kinesis"   # Symlink for the Kinesis Motor (Goniometer)
# ---------------------

def main_menu(aligner):
    """
    Displays an interactive menu for direct manual control of the stepper motors.
    Allows for quick switching between axes.
    """
    active_axis = 'x'  # Start with the X-axis as the active one

    while True:
        print("\n--- Manual Steering Control ---")
        # Display live readings to aid manual alignment
        try:
            r = aligner.kqd.get_readings()
            print(f"Live Position | X: {r.xdiff:.5f}, Y: {r.ydiff:.5f}")
        except Exception as e:
            print(f"Could not get live reading: {e}")

        print(f"** Active Axis: {active_axis.upper()} **")
        print("Enter a distance (e.g., 100 or -50) to move the active axis.")
        print("Type 's' to switch axis, 'q' to quit.")
        
        command = input("Input: ").lower().strip()

        if command == 'q':
            print("Exiting manual control.")
            break
        elif command == 's':
            active_axis = 'y' if active_axis == 'x' else 'x'
            print(f"--> Switched to {active_axis.upper()}-axis.")
            continue
        
        try:
            distance = int(command)
            if active_axis == 'x':
                aligner.manual_x(distance, relative=True)
            else:  # active_axis must be 'y'
                aligner.manual_y(distance, relative=True)
        except ValueError:
            print("Invalid input. Please enter a whole number, 's', or 'q'.")
        except Exception as e:
            print(f"An error occurred during move: {e}")


def main():
    """
    Main function to initialize and run the manual steering interface.
    """
    print("--- Starting Manual Steering Routine ---")
    aligner = OpticalAligner(kqd_port=KQD_PORT, km_port=KM_PORT)

    try:
        # Connect to hardware needed for manual slow steering
        aligner.connect_slow_steering()
        # Start the interactive menu
        main_menu(aligner)

    except Exception as e:
        print(f"\nA critical error occurred: {e}")
        sys.exit(1)
    finally:
        print("\nApplication shutting down.")
        aligner.disconnect_slow_steering()

if __name__ == "__main__":
    main()

