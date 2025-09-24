import sys
import time
from optical_aligner import OpticalAligner
from usbfinder import USBTTYFinder

# --- CONFIGURATION ---
usb = USBTTYFinder()
KQD_PORT = usb.find_by_product("Position Aligner")[0]
KM_PORT = usb.find_by_product("Brushed Motor Controller")[0]
# ---------------------

def manage_soft_limits(aligner):
    """Interactive menu for managing soft limits for both axes."""
    while True:
        print("\n--- Soft Limits Management ---")
        print("1. Set X-Axis Limits")
        print("2. Clear X-Axis Limits")
        print("3. Set Y-Axis Limits")
        print("4. Clear Y-Axis Limits")
        print("5. Return to Main Menu")
        choice = input("Select an option: ")
        try:
            if choice == '1':
                current_pos = aligner.tic.get_current_position()
                range_val = int(input(f"  Enter range (+/-) from current X position {current_pos}: "))
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                aligner.set_tic_soft_limits(min_pos, max_pos)
                print(f"X-Axis soft limits set to ({min_pos}, {max_pos})")
            elif choice == '2':
                aligner.clear_tic_soft_limits()
                print("X-Axis soft limits cleared.")
            elif choice == '3':
                current_pos = aligner.km.get_position()
                range_val = int(input(f"  Enter range (+/-) from current Y position {current_pos}: "))
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                aligner.set_km_soft_limits(min_pos, max_pos)
                print(f"Y-Axis soft limits set to ({min_pos}, {max_pos})")
            elif choice == '4':
                aligner.clear_km_soft_limits()
                print("Y-Axis soft limits cleared.")
            elif choice == '5':
                break
            else:
                print("Invalid option.")
        except Exception as e:
            print(f"An error occurred: {e}", file=sys.stderr)

def main():
    """Main function to run the slow optical aligner."""
    print("--- Starting Slow Steering Routine ---")
    try:
        aligner = OpticalAligner(kqd_port=KQD_PORT, km_port=KM_PORT)
        aligner.connect_slow_steering()

        while True:
            print("\n--- Main Menu ---")
            print("1. Manage Settings")
            print("2. Manage Soft Limits")
            print("3. Run Calibration")
            print("4. Run Feedback Loop")
            print("5. Recalibrate Signal Baseline")
            print("6. Exit")
            choice = input("Select an option: ")

            if choice == '1':
                aligner.manage_slow_steering_settings()
            elif choice == '2':
                manage_soft_limits(aligner)
            elif choice == '3':
                aligner.calibrate()
            elif choice == '4':
                try:
                    gain_str = input(f"Enter master loop gain [default: 1.0]: ")
                    loop_gain = float(gain_str) if gain_str else 1.0
                except ValueError:
                    print("Invalid input. Using default gain of 1.0.")
                    loop_gain = 1.0
                aligner.run_slow_feedback_loop(loop_gain=loop_gain)
            elif choice == '5':
                aligner.recalibrate_sum_baseline()
            elif choice == '6':
                break
            else:
                print("Invalid option.")

    except Exception as e:
        print(f"\nA critical error occurred: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        print("\nApplication shutting down.")
        if 'aligner' in locals() and aligner.km:
            aligner.disconnect_slow_steering()

if __name__ == "__main__":
    main()

