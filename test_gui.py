import tkinter as tk
from tkinter import ttk
import sys

def main():
    """
    A minimal script to test if a basic Tkinter window can be created.
    """
    print("--- GUI Environment Test ---")
    print("Attempting to create a basic GUI window...")
    
    try:
        # 1. Create the main window object
        root = tk.Tk()
        root.title("GUI Environment Test")
        root.geometry("400x150")

        # 2. Create a simple widget
        label = ttk.Label(root, text="If you can see this window, your GUI environment is working correctly.", wraplength=380)
        label.pack(pady=20, padx=20)

        # 3. Create a button that closes the window
        button = ttk.Button(root, text="Quit", command=root.destroy)
        button.pack(pady=10)

        print("Window created successfully. Starting the GUI...")
        
        # 4. Start the GUI event loop
        root.mainloop()
        
        print("GUI window closed successfully.")

    except tk.TclError as e:
        print("\n--- A CRITICAL ERROR OCCURRED ---")
        print("Failed to create the GUI window. Could not connect to a graphical display.")
        print("This is the most common cause for a 'silent exit'.")
        print("\nACTION: If you are using SSH, please ensure you have connected with X11 forwarding enabled.")
        print("        The command is: ssh -X pi@<your_pi_ip_address>")
        print(f"\nError details: {e}")
        print("---------------------------------")
        sys.exit(1) # Exit with an error code
    except Exception as e:
        print(f"\n--- AN UNEXPECTED ERROR OCCURRED ---")
        print(f"Error details: {e}")
        print("------------------------------------")
        sys.exit(1)


if __name__ == "__main__":
    main()
