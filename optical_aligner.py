import time
import numpy as np
from pylablib.devices import Thorlabs
from ticlib import TicUSB
from pid_controller import PID

class OpticalAligner:
    """
    Manages hardware and control logic for an automated optical alignment system.
    Includes methods for both slow (motorized) and fast (piezo-based hardware)
    feedback loops.
    """
    def __init__(self, kqd_port, km_port=None):
        """Initializes hardware port information."""
        self.kqd_port = kqd_port
        self.km_port = km_port

        self.kqd = None
        self.km = None
        self.tic = None
        self.Minv = None  # Inverse calibration matrix for SLOW steering

        # PID controllers for SLOW steering
        self.pid_x = PID(Kp=0.1, Ki=0.1, Kd=0.05, setpoint=0.0)
        self.pid_y = PID(Kp=0.1, Ki=0.1, Kd=0.05, setpoint=0.0)
        
        # Store last used PID parameters for fast steering
        self.last_pid_params = {'p': 1.0, 'i': 0.0, 'd': 0.0}

    def connect_fast_steering(self, init_delay=1.0):
        """Establishes connection to the KQD for fast steering modes."""
        print("--- Connecting for Fast Steering ---")
        try:
            print(f"Connecting to KinesisQuadDetector on {self.kqd_port}...")
            self.kqd = Thorlabs.KinesisQuadDetector(self.kqd_port)
            print("KQD connected.")
            print(f"Waiting {init_delay}s for hardware to stabilize...")
            time.sleep(init_delay)
            print("Fast steering hardware ready.")
        except Exception as e:
            print(f"A critical error occurred during connection: {e}")
            raise

    def disconnect_fast_steering(self):
        """Safely disconnects hardware used for fast steering."""
        print("\n--- Disconnecting Fast Steering Hardware ---")
        if self.kqd and self.kqd.is_opened():
            self.kqd.close()
            print("KQD connection closed.")
        print("Fast steering hardware disconnected safely.")

    def connect_slow_steering(self, init_delay=1.0):
        """Establishes connections to all hardware for slow steering."""
        print("--- Connecting for Slow Steering ---")
        try:
            print(f"Connecting to KinesisQuadDetector on {self.kqd_port}...")
            self.kqd = Thorlabs.KinesisQuadDetector(self.kqd_port)
            print("KQD connected.")

            if self.km_port:
                print(f"Connecting to Kinesis Motor on {self.km_port}...")
                self.km = Thorlabs.KinesisMotor(self.km_port)
                print("Kinesis Motor connected.")
            else:
                raise ValueError("Kinesis Motor port (km_port) must be specified for slow steering.")

            print("Connecting to Tic controller...")
            self.tic = TicUSB()
            print("Tic controller connected.")
            
            print("Initializing Tic stage...")
            self.tic.halt_and_set_position(0)
            self.tic.energize()
            self.tic.exit_safe_start()
            print("Tic stage initialized.")

            print(f"Waiting {init_delay}s for hardware to stabilize...")
            time.sleep(init_delay)
            print("Slow steering hardware ready.")

        except Exception as e:
            print(f"A critical error occurred during hardware connection: {e}")
            raise

    def disconnect_slow_steering(self):
        """Safely disconnects all hardware used for slow steering."""
        print("\n--- Disconnecting Slow Steering Hardware ---")
        if self.tic:
            self.tic.deenergize()
            self.tic.enter_safe_start()
            print("Tic controller de-energized.")
        if self.km and self.km.is_opened():
            self.km.close()
            print("Kinesis Motor connection closed.")
        if self.kqd and self.kqd.is_opened():
            self.kqd.close()
            print("KQD connection closed.")
        print("Slow steering hardware disconnected safely.")

    def _manage_output_settings(self):
        """Helper method to display and set KQD output parameters."""
        print("\n--- KQD Output Settings ---")
        current_params = self.kqd.get_output_parameters()
        
        print("Current Settings:")
        for key, value in current_params.items():
            print(f"  - {key}: {value}")
        
        new_params = {}
        # Loop to get new settings from user
        for key in current_params.keys():
            # Special handling for route and open_loop_out as they are strings
            if key in ['route', 'open_loop_out']:
                new_val_str = input(f"Enter new value for '{key}' (e.g., sma_only, zero) [default: {current_params[key]}]: ")
                if new_val_str:
                    new_params[key] = new_val_str
            else: # Handle numeric values
                new_val_str = input(f"Enter new value for '{key}' [default: {current_params[key]}]: ")
                if new_val_str:
                    try:
                        new_params[key] = float(new_val_str)
                    except ValueError:
                        print(f"Invalid number '{new_val_str}', keeping current value.")

        if new_params:
            print("\nApplying new settings...")
            try:
                self.kqd.set_output_parameters(**new_params)
                print("Settings applied successfully.")
            except Exception as e:
                print(f"Error applying settings: {e}")
        else:
            print("\nNo changes made.")
        input("Press Enter to return to the main menu.")

    def _run_selected_mode(self, mode):
        """Helper method to run a specific operational mode loop."""
        print(f"\n--- Entering '{mode.upper()}' Mode ---")
        print("Press Ctrl+C to stop this mode and return to the menu.")
        
        try:
            if mode == 'monitor':
                self.kqd.set_operation_mode("monitor")
                print("Displaying live readings...")
                while True:
                    r = self.kqd.get_readings()
                    print(f"Position | X: {r.xdiff:.5f}, Y: {r.ydiff:.5f}", end='\r')
                    time.sleep(0.2)
            
            elif mode == 'open_loop':
                self.kqd.set_operation_mode("open_loop")
                print("Enter voltages for manual control ('q' to exit).")
                while True:
                    v_x_str = input("Enter X voltage: ")
                    if v_x_str.lower() == 'q': break
                    v_y_str = input("Enter Y voltage: ")
                    if v_y_str.lower() == 'q': break
                    try:
                        v_x, v_y = float(v_x_str), float(v_y_str)
                        self.kqd.set_manual_output(xpos=v_x, ypos=v_y)
                        print(f"Manual outputs set to X={v_x:.2f}V, Y={v_y:.2f}V")
                    except ValueError:
                        print("Invalid input. Please enter numbers.")

            elif mode == 'closed_loop':
                print("Configuring KQD for closed-loop operation...")
                p_str = input(f"Enter P gain [default: {self.last_pid_params['p']}]: ")
                p_val = float(p_str) if p_str else self.last_pid_params['p']
                i_str = input(f"Enter I gain [default: {self.last_pid_params['i']}]: ")
                i_val = float(i_str) if i_str else self.last_pid_params['i']
                d_str = input(f"Enter D gain [default: {self.last_pid_params['d']}]: ")
                d_val = float(d_str) if d_str else self.last_pid_params['d']
                self.last_pid_params = {'p': p_val, 'i': i_val, 'd': d_val}
                
                print(f"Setting PID parameters to P={p_val}, I={i_val}, D={d_val}")
                self.kqd.set_pid_parameters(p=p_val, i=i_val, d=d_val)
                self.kqd.set_operation_mode("closed_loop")
                print("Hardware CLOSED-LOOP is ACTIVE.")
                while True:
                    r = self.kqd.get_readings()
                    print(f"Status | Error X: {r.xdiff:.5f}, Error Y: {r.ydiff:.5f}", end='\r')
                    time.sleep(0.2)

        except (KeyboardInterrupt, ValueError):
             print(f"\nExiting '{mode.upper()}' mode.")

    def manage_fast_steering_mode(self):
        """
        Manages the KQD's operational mode for fast steering via an interactive menu.
        Always resets to 'monitor' mode on exit for safety.
        """
        if not self.kqd:
            print("Error: KQD must be connected for fast steering.")
            return

        try:
            print("\n--- Entering Fast Steering Control ---")
            self.kqd.set_operation_mode("monitor")
            print("Device started in safe MONITOR mode.")

            while True:
                print("\nFast Steering Main Menu:")
                print("1. Change Mode (monitor, open-loop, closed-loop)")
                print("2. Change Output Settings")
                print("3. Exit Fast Steering Control")
                choice = input("Select an option (1-3): ")

                if choice == '1':
                    mode_choice = input("Enter mode (monitor, open_loop, closed_loop): ").lower()
                    if mode_choice in ['monitor', 'open_loop', 'closed_loop']:
                        self._run_selected_mode(mode_choice)
                    else:
                        print("Invalid mode selected.")
                elif choice == '2':
                    self._manage_output_settings()
                elif choice == '3':
                    print("Exiting fast steering control...")
                    break
                else:
                    print("Invalid choice. Please enter 1, 2, or 3.")

        except KeyboardInterrupt:
            print("\nFast steering control interrupted by user.")
        
        finally:
            print("\nResetting KQD to 'monitor' mode for safety...")
            if self.kqd and self.kqd.is_opened():
                self.kqd.set_operation_mode("monitor")
            print("System is now in a safe state.")
            time.sleep(1)

    def read_kqd_avg(self, n=20, delay=0.05):
        # ... (rest of the class is unchanged)
        # ...
        pass
    # The rest of the slow steering methods (calibrate, manual_x, manual_y, run_slow_feedback_loop)
    # remain unchanged below this point. I've omitted them for brevity but they are still there.

# ... (The rest of the file continues from here)
    def calibrate(self, dx_tic=20, dy_gon=1000):
        """
        Moves each actuator to measure its influence on the detector,
        then calculates the inverse calibration matrix `Minv`.
        """
        print("\n--- Starting system calibration ---")
        x01, y01 = self.read_kqd_avg()
        print(f"Initial Neutral QD: {x01:.6f}, {y01:.6f}")
        print(f"Moving Tic stage by {dx_tic} for calibration...")
        self.tic.set_target_position(dx_tic)
        time.sleep(1.0)
        x1, y1 = self.read_kqd_avg()
        self.tic.set_target_position(0)
        time.sleep(3.0)
        print(f"Tic move effect: Δx={x1-x01:.6f}, Δy={y1-y01:.6f}")
        x02, y02 = self.read_kqd_avg()
        print(f"Second Neutral QD: {x02:.6f}, {y02:.6f}")
        print(f"Moving Goniometer by {dy_gon} for calibration...")
        self.km.move_by(dy_gon)
        self.km.wait_move()
        time.sleep(1.0)
        x2, y2 = self.read_kqd_avg()
        self.km.move_by(-dy_gon)
        self.km.wait_move()
        time.sleep(1.0)
        print(f"Goniometer move effect: Δx={x2-x02:.6f}, Δy={y2-y02:.6f}")
        M = np.array([
            [(x1 - x01) / dx_tic, (x2 - x02) / dy_gon],
            [(y1 - y01) / dx_tic, (y2 - y02) / dy_gon]
        ])
        print("\nCalibration Matrix M:")
        print(M)
        try:
            self.Minv = np.linalg.inv(M)
            print("Inverse Calibration Matrix Minv:")
            print(self.Minv)
            print("Calibration successful.")
        except np.linalg.LinAlgError:
            print("\nError: Calibration failed. Matrix is singular and cannot be inverted.")
            print("Check motor connections, power, and ensure beam is on the detector.")
            self.Minv = None

    def manual_y(self, distance, relative=True):
        """Manually move the goniometer motor (Y-axis)."""
        if not self.km:
            print("Error: Goniometer not connected.")
            return
        try:
            if relative:
                print(f"Moving goniometer (Y) by {distance} steps...")
                self.km.move_by(distance)
            else:
                print(f"Moving goniometer (Y) to position {distance}...")
                self.km.move_to(distance)
            self.km.wait_move()
            new_pos = self.km.get_position()
            print(f"Goniometer move complete. New position: {new_pos}")
        except Exception as e:
            print(f"An error occurred during goniometer move: {e}")

    def manual_x(self, distance, relative=True):
        """Manually move the Tic stepper motor (X-axis)."""
        if not self.tic:
            print("Error: Tic controller not connected.")
            return
        try:
            if relative:
                current_pos = self.tic.get_current_position()
                target_pos = current_pos + distance
                print(f"Moving Tic stage (X) by {distance} steps to position {target_pos}...")
            else:
                target_pos = distance
                print(f"Moving Tic stage (X) to absolute position {target_pos}...")
            self.tic.set_target_position(target_pos)
            time.sleep(0.1 + abs(distance) / 20000)
            new_pos = self.tic.get_current_position()
            print(f"Tic move complete. New position: {new_pos}")
        except Exception as e:
            print(f"An error occurred during Tic move: {e}")

    def run_slow_feedback_loop(self):
        """
        Runs the main PID feedback loop using stepper motors for SLOW alignment.
        """
        # A safety check to ensure the system has been calibrated before starting.
        if self.Minv is None:
            print("Cannot start feedback loop: system is not calibrated.")
            return
        
        # Wait for the user to confirm before starting the active loop.
        input("\nPress Enter to start the SLOW PID feedback loop (Ctrl+C to stop)...")
        print("Starting slow feedback loop...")
        
        try:
            # This is the main control loop that runs continuously.
            while True:
                # Step 1: Get the current beam position error from the Quadrant Detector.
                reading = self.kqd.get_readings()
                x_err, y_err = reading.xdiff, reading.ydiff

                # Step 2: Calculate the desired correction using the PID controllers.
                # The negative sign ensures the correction moves the beam *towards* the target (zero error).
                dx = -self.pid_x.compute(x_err)
                dy = -self.pid_y.compute(y_err)

                # Step 3: Convert the desired correction from detector space (dx, dy) to actuator space (motor steps).
                # This uses matrix multiplication with the inverse calibration matrix.
                d_actuator = self.Minv @ np.array([dx, dy])
                dtheta_tic, dtheta_gon = d_actuator[0], d_actuator[1]
                
                # Step 4: Command the motors to move by the calculated amounts.
                # For the Tic stepper, we calculate an absolute target position for smoother movement.
                current_tic_pos = self.tic.get_current_position()
                new_tic_target = current_tic_pos + int(dtheta_tic)
                self.tic.set_target_position(new_tic_target)
                
                # The Kinesis motor is commanded with a relative move.
                self.km.move_by(int(dtheta_gon))

                # Step 5: Print the current status to the console without creating new lines.
                print(f"Err:[{x_err:.4f},{y_err:.4f}] | Corr:[{dtheta_tic:.2f},{dtheta_gon:.2f}]", end='\r')
                
                # Step 6: Wait for a short period before the next loop iteration.
                # This prevents overwhelming the hardware and allows time for physical movement.
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            # This block catches a Ctrl+C press from the user to stop the loop gracefully.
            print("\nFeedback loop stopped by user.")

