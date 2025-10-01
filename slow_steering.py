import sys
import time
from optical_aligner import OpticalAligner
from usbfinder import USBTTYFinder

# --- CONFIGURATION ---
try:
    usb = USBTTYFinder()
    KQD_PORT = usb.find_by_product("Position Aligner")[0]
    KM_PORT = usb.find_by_product("Brushed Motor Controller")[0]
except Exception as e:
    print(f"CRITICAL ERROR: Could not find required hardware. Check connections.")
    print(f"Details: {e}")
    sys.exit(1)
# ---------------------

def manage_soft_limits(aligner):
    """Interactive menu for managing soft limits for both axes."""
    while True:
        print("\n--- Soft Limits Management ---")
        print("1. Set X-Axis Limits\n2. Clear X-Axis Limits\n3. Set Y-Axis Limits\n4. Clear Y-Axis Limits\n5. Return to Main Menu")
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
            print("3. Calibrate X-Axis")
            print("4. Calibrate Y-Axis")
            print("5. Recalibrate Signal Baseline")
            print("6. Run Feedback Loop")
            print("7. Save Calibration")
            print("8. Load Calibration")
            print("9. Clear Calibration")
            print("10. Exit")
            choice = input("Select an option: ")

            if choice == '1':
                aligner.manage_slow_steering_settings()
            elif choice == '2':
                manage_soft_limits(aligner)
            elif choice == '3':
                aligner.calibrate_x_axis()
            elif choice == '4':
                aligner.calibrate_y_axis()
            elif choice == '5':
                aligner.recalibrate_sum_baseline()
            elif choice == '6':
                try:
                    gain_x_str = input(f"Enter X-Axis loop gain [default: 1.0]: ")
                    loop_gain_x = float(gain_x_str) if gain_x_str else 1.0
                    gain_y_str = input(f"Enter Y-Axis loop gain [default: 1.0]: ")
                    loop_gain_y = float(gain_y_str) if gain_y_str else 1.0
                except ValueError:
                    print("Invalid input. Using default gains of 1.0.")
                    loop_gain_x, loop_gain_y = 1.0, 1.0
                aligner.run_slow_feedback_loop(loop_gain_x=loop_gain_x, loop_gain_y=loop_gain_y)
            elif choice == '7':
                filepath = input("Enter filename to save calibration (e.g., cal_data.json): ")
                aligner.save_calibration_to_file(filepath)
            elif choice == '8':
                filepath = input("Enter filename to load calibration from: ")
                aligner.load_calibration_from_file(filepath)
            elif choice == '9':
                if input("Are you sure you want to clear all calibration data? (y/n): ").lower() == 'y':
                    aligner.clear_calibration_data()
            elif choice == '10':
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

