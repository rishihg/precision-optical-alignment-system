import sys
from optical_aligner import OpticalAligner
from pylablib.devices import Thorlabs

# --- CONFIGURATION ---
# UPDATE THESE TO MATCH YOUR SYSTEM'S DEVICE PATHS
KQD_PORT = "/dev/ttyUSB0" # Example for Linux
KM_PORT = "/dev/ttyUSB1"  # Example for Linux
# ---------------------

def main_menu(aligner):
    """
    Displays an interactive menu for direct manual control of the stepper motors.
    """
    active_axis = 'x'

    while True:
        print("\n--- Manual Steering Control ---")
        try:
            r = aligner.kqd.get_readings()
            if r:
                print(f"Live Position | X: {r.xdiff:.5f}, Y: {r.ydiff:.5f}, Sum: {r.sum:.4f}")
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
                print(f"Moved X-axis by {distance} steps.")
            else:
                aligner.manual_y(distance, relative=True)
                print(f"Moved Y-axis by {distance} steps.")
        except ValueError:
            print("Invalid input. Please enter a whole number, 's', or 'q'.")
        except Exception as e:
            print(f"An error occurred during move: {e}")

def main():
    """
    Main function to initialize and run the manual steering interface.
    """
    print("--- Starting Manual Steering Routine ---")
    try:
        aligner = OpticalAligner(kqd_port=KQD_PORT, km_port=KM_PORT)
        aligner.connect_slow_steering()
        main_menu(aligner)

    except Exception as e:
        print(f"\nA critical error occurred: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        print("\nApplication shutting down.")
        if 'aligner' in locals() and aligner.km:
            aligner.disconnect_slow_steering()

if __name__ == "__main__":
    main()

