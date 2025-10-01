import sys
from optical_aligner import OpticalAligner
from usbfinder import USBTTYFinder

# --- CONFIGURATION ---
# This script only requires the KQD controller.
try:
    usb = USBTTYFinder()
    KQD_PORT = usb.find_by_product("Position Aligner")[0]
except Exception as e:
    print(f"CRITICAL ERROR: Could not find KQD hardware (Position Aligner). Check connections.")
    print(f"Details: {e}")
    sys.exit(1)
# ---------------------

def main():
    """
    Main function to initialize and run the fast steering interactive menu.
    """
    print("--- Starting Fast Steering Routine ---")
    print(f"Using KQD Controller on port: {KQD_PORT}")

    # The aligner is initialized without the optional km_port
    aligner = OpticalAligner(kqd_port=KQD_PORT)

    try:
        # Connect to all available hardware (in this case, just the KQD)
        aligner.connect_all()
        
        # Launch the interactive command-line menu
        aligner.manage_fast_steering_mode()

    except Exception as e:
        print(f"\nA critical error occurred: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        print("\nApplication shutting down.")
        # Ensure the aligner object exists before trying to disconnect
        if 'aligner' in locals() and aligner.kqd:
            aligner.disconnect_all()

if __name__ == "__main__":
    main()

