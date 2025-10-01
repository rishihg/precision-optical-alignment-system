import sys
from optical_aligner import OpticalAligner
from usbfinder import USBTTYFinder

# --- CONFIGURATION ---
try:
    usb = USBTTYFinder()
    # Fast steering only requires the KQD controller
    KQD_PORT = usb.find_by_product("Position Aligner")[0]
except Exception as e:
    print(f"CRITICAL ERROR: Could not find KQD hardware. Check connections.")
    print(f"Details: {e}")
    sys.exit(1)
# ---------------------

def main():
    """
    Main function to initialize and run the fast steering interactive menu.
    """
    print("--- Starting Fast Steering Routine ---")
    print(f"Using KQD Controller on port: {KQD_PORT}")

    # The aligner only needs the KQD port for fast steering
    aligner = OpticalAligner(kqd_port=KQD_PORT)

    try:
        # Connect to only the fast steering hardware
        aligner.connect_fast_steering()
        
        # Launch the interactive command-line menu
        aligner.manage_fast_steering_mode()

    except Exception as e:
        print(f"\nA critical error occurred: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        print("\nApplication shutting down.")
        # Ensure the aligner object exists before trying to disconnect
        if 'aligner' in locals() and aligner.kqd:
            aligner.disconnect_fast_steering()

if __name__ == "__main__":
    main()

