import time
import numpy as np
from pylablib.devices import Thorlabs
from ticlib import TicUSB
from pid_controller import PID
import sys
import threading

class OpticalAligner:
    """
    Manages hardware and control logic for an automated optical alignment system.
    """
    def __init__(self, kqd_port=None, km_port=None):
        self.kqd_port = kqd_port
        self.km_port = km_port
        self.kqd = None
        self.km = None
        self.tic = None
        self.Minv = None
        self.pid_x = PID(Kp=0.1, Ki=0.1, Kd=0.05, setpoint=0.0)
        self.pid_y = PID(Kp=0.1, Ki=0.1, Kd=0.05, setpoint=0.0)
        self.baseline_sum = None
        self.last_pid_params = {'p': 1.0, 'i': 0.0, 'd': 0.0}
        self.stop_loop_event = threading.Event()

    def connect_fast_steering(self):
        if not self.kqd_port:
            raise ValueError("KQD port (serial number) is not set.")
        print("--- Connecting to KQD for fast steering ---")
        self.kqd = Thorlabs.KinesisQuadDetector(self.kqd_port)
        print("KQD connected successfully.")

    def disconnect_fast_steering(self):
        print("\n--- Disconnecting KQD ---")
        if self.kqd and self.kqd.is_opened():
            try:
                self.kqd.set_operation_mode("monitor")
                self.kqd.close()
            except Exception as e:
                print(f"Could not fully disconnect KQD: {e}", file=sys.stderr)
        self.kqd = None
        print("KQD disconnected.")

    def connect_slow_steering(self):
        if not self.kqd_port or not self.km_port:
            raise ValueError("KQD and Kinesis Motor ports are not set.")
        print("--- Connecting for Slow Steering ---")
        print(f"Connecting to KinesisQuadDetector on {self.kqd_port}...")
        self.kqd = Thorlabs.KinesisQuadDetector(self.kqd_port)
        print("KQD connected.")
        print(f"Connecting to Kinesis Motor on {self.km_port}...")
        self.km = Thorlabs.KinesisMotor(self.km_port)
        print("Kinesis Motor connected.")
        print("Connecting to Tic controller...")
        self.tic = TicUSB()
        print("Tic controller connected.")
        print("Initializing Tic stage...")
        self.tic.halt_and_set_position(0)
        self.tic.energize()
        self.tic.exit_safe_start()
        print("Tic stage initialized.")
        print("Waiting 1.0s for hardware to stabilize...")
        time.sleep(1.0)
        print("Slow steering hardware ready.")

    def disconnect_slow_steering(self):
        print("\n--- Disconnecting Slow Steering Hardware ---")
        if self.tic:
            self.tic.deenergize()
            print("Tic controller de-energized.")
        if self.km and self.km.is_opened():
            self.km.close()
            print("Kinesis Motor connection closed.")
        if self.kqd and self.kqd.is_opened():
            self.kqd.close()
            print("KQD connection closed.")
        self.kqd, self.km, self.tic = None, None, None
        print("Slow steering hardware disconnected safely.")

    def manage_fast_steering_mode(self):
        if not self.kqd:
            print("Error: Fast steering hardware not connected.", file=sys.stderr)
            return
        print("\n--- Fast Steering Control ---")
        self.kqd.set_operation_mode("monitor")
        try:
            while True:
                print("\n--- Main Menu ---")
                current_mode = self.kqd.get_operation_mode()
                print(f"Current Mode: {current_mode.upper()}")
                print("1. Change Mode")
                print("2. Change Settings")
                print("3. Exit (resets to monitor mode)")
                choice = input("Select an option: ").strip()
                if choice == '1':
                    self._run_selected_mode()
                elif choice == '2':
                    self._manage_output_settings()
                elif choice == '3':
                    break
                else:
                    print("Invalid option. Please try again.")
        finally:
            print("\nResetting KQD to 'monitor' mode before exiting.")
            self.kqd.set_operation_mode("monitor")

    def _run_selected_mode(self):
        mode_choice = input("Select mode (monitor, open_loop, closed_loop): ").lower().strip()
        if mode_choice not in ["monitor", "open_loop", "closed_loop"]:
            print("Invalid mode.")
            return
        try:
            if mode_choice == "closed_loop":
                print("\nEnter PID values. Press Enter to use the last known values.")
                p_str = input(f"  Enter P gain [default: {self.last_pid_params['p']}]: ")
                i_str = input(f"  Enter I gain [default: {self.last_pid_params['i']}]: ")
                d_str = input(f"  Enter D gain [default: {self.last_pid_params['d']}]: ")
                p_val = float(p_str) if p_str else self.last_pid_params['p']
                i_val = float(i_str) if i_str else self.last_pid_params['i']
                d_val = float(d_str) if d_str else self.last_pid_params['d']
                self.last_pid_params = {'p': p_val, 'i': i_val, 'd': d_val}
                self.kqd.set_pid_parameters(p=p_val, i=i_val, d=d_val)
                self.kqd.set_operation_mode("closed_loop")
            elif mode_choice == "open_loop":
                x_pos = float(input("  Enter manual X output voltage (-10 to 10): "))
                y_pos = float(input("  Enter manual Y output voltage (-10 to 10): "))
                self.kqd.set_manual_output(xpos=x_pos, ypos=y_pos)
                self.kqd.set_operation_mode("open_loop")
            elif mode_choice == "monitor":
                self.kqd.set_operation_mode("monitor")
            print(f"\nMode set to '{mode_choice}'. Press Ctrl+C to return to the main menu.")
            while True:
                r = self.kqd.get_readings()
                print(f"X_diff: {r.xdiff:.4f}, Y_diff: {r.ydiff:.4f}, Sum: {r.sum:.4f}", end='\r')
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nReturning to main menu.")
        except Exception as e:
            print(f"An error occurred: {e}", file=sys.stderr)

    def _manage_output_settings(self):
        try:
            current = self.kqd.get_output_parameters()
            print("\n--- Current Output Settings ---")
            for key, value in current.items():
                print(f"  - {key}: {value}")
            if input("\nUpdate settings? (y/n): ").lower() != 'y':
                return
            print("\nEnter new value or press Enter to keep current setting.")
            xmin = float(input(f"  xmin [{current['xmin']}]: ") or current['xmin'])
            xmax = float(input(f"  xmax [{current['xmax']}]: ") or current['xmax'])
            ymin = float(input(f"  ymin [{current['ymin']}]: ") or current['ymin'])
            ymax = float(input(f"  ymax [{current['ymax']}]: ") or current['ymax'])
            xgain = float(input(f"  xgain [{current['xgain']}]: ") or current['xgain'])
            ygain = float(input(f"  ygain [{current['ygain']}]: ") or current['ygain'])
            route = input(f"  route ('sma_only' or 'sma_hub') [{current['route']}]: ") or current['route']
            open_loop_out = input(f"  open_loop_out ('zero' or 'fixed') [{current['open_loop_out']}]: ") or current['open_loop_out']
            self.kqd.set_output_parameters(
                xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax,
                xgain=xgain, ygain=ygain, route=route, open_loop_out=open_loop_out
            )
            print("\nSettings updated successfully.")
        except Exception as e:
            print(f"\nAn error occurred while managing settings: {e}", file=sys.stderr)

    def get_pid_settings(self):
        return {
            'x': {'p': self.pid_x.Kp, 'i': self.pid_x.Ki, 'd': self.pid_x.Kd},
            'y': {'p': self.pid_y.Kp, 'i': self.pid_y.Ki, 'd': self.pid_y.Kd}
        }

    def set_pid_settings(self, pid_x_p, pid_x_i, pid_x_d, pid_y_p, pid_y_i, pid_y_d):
        self.pid_x.Kp, self.pid_x.Ki, self.pid_x.Kd = pid_x_p, pid_x_i, pid_x_d
        self.pid_y.Kp, self.pid_y.Ki, self.pid_y.Kd = pid_y_p, pid_y_i, pid_y_d

    def get_tic_settings(self):
        if not self.tic: raise ConnectionError("Tic not connected.")
        return {
            'velocity': self.tic.get_max_speed(),
            'current_limit': self.tic.get_current_limit(),
            'step_mode': self.tic.get_step_mode()
        }

    def set_tic_settings(self, velocity, current_limit, step_mode):
        if not self.tic: raise ConnectionError("Tic not connected.")
        self.tic.set_max_speed(velocity)
        self.tic.set_current_limit(current_limit)
        self.tic.set_step_mode(step_mode)

    def get_km_settings(self):
        if not self.km: raise ConnectionError("Kinesis Motor not connected.")
        vel_params = self.km.get_velocity_parameters()
        return {'velocity': vel_params.max_velocity}

    def set_km_settings(self, velocity):
        if not self.km: raise ConnectionError("Kinesis Motor not connected.")
        current_accel = self.km.get_velocity_parameters().acceleration
        self.km.set_velocity_parameters(max_velocity=velocity, acceleration=current_accel)

    def manage_slow_steering_settings(self):
        if not self.tic or not self.km:
            print("Error: Slow steering hardware not fully connected.", file=sys.stderr)
            return
        print("\n--- Slow Steering Settings ---")
        try:
            pid = self.get_pid_settings()
            tic = self.get_tic_settings()
            km = self.get_km_settings()
            print("\nCurrent Settings:")
            print(f"  - X-Axis PID (P/I/D):      {pid['x']['p']}, {pid['x']['i']}, {pid['x']['d']}")
            print(f"  - Y-Axis PID (P/I/D):      {pid['y']['p']}, {pid['y']['i']}, {pid['y']['d']}")
            print(f"  - X-Axis Max Velocity:     {tic['velocity']} (microsteps/10000s)")
            print(f"  - Y-Axis Max Velocity:     {km['velocity']} (device units)")
            print(f"  - X-Axis Current Limit:    {tic['current_limit']} (mA)")
            print(f"  - X-Axis Step Mode:        {tic['step_mode']}")
            if input("\nUpdate settings? (y/n): ").lower() != 'y':
                return
            print("\nEnter new value or press Enter to keep current setting.")
            new_px = float(input(f"  New X P-gain [{pid['x']['p']}]: ") or pid['x']['p'])
            new_ix = float(input(f"  New X I-gain [{pid['x']['i']}]: ") or pid['x']['i'])
            new_dx = float(input(f"  New X D-gain [{pid['x']['d']}]: ") or pid['x']['d'])
            new_py = float(input(f"  New Y P-gain [{pid['y']['p']}]: ") or pid['y']['p'])
            new_iy = float(input(f"  New Y I-gain [{pid['y']['i']}]: ") or pid['y']['i'])
            new_dy = float(input(f"  New Y D-gain [{pid['y']['d']}]: ") or pid['y']['d'])
            self.set_pid_settings(new_px, new_ix, new_dx, new_py, new_iy, new_dy)
            new_tic_vel = int(input(f"  New X-Axis Velocity [{tic['velocity']}]: ") or tic['velocity'])
            new_tic_curr = int(input(f"  New X-Axis Current [{tic['current_limit']}]: ") or tic['current_limit'])
            new_tic_step = int(input(f"  New X-Axis Step Mode (0-5) [{tic['step_mode']}]: ") or tic['step_mode'])
            self.set_tic_settings(new_tic_vel, new_tic_curr, new_tic_step)
            new_km_vel = float(input(f"  New Y-Axis Velocity [{km['velocity']}]: ") or km['velocity'])
            self.set_km_settings(new_km_vel)
            print("\nSettings updated successfully.")
        except Exception as e:
            print(f"\nAn error occurred during settings update: {e}", file=sys.stderr)

    def read_kqd_avg(self, n=20, delay=0.05):
        if not self.kqd:
            print("Error: KQD not connected, can't take reading.", file=sys.stderr)
            return None, None, None
        xs, ys, sums = [], [], []
        for i in range(n):
            try:
                r = self.kqd.get_readings()
                if r is not None:
                    xs.append(r.xdiff)
                    ys.append(r.ydiff)
                    sums.append(r.sum)
                else:
                    pass
                time.sleep(delay)
            except Exception as e:
                print(f"Error during KQD read on attempt {i+1}/{n}: {e}", file=sys.stderr)
        if not xs or not ys:
            return None, None, None
        return np.mean(xs), np.mean(ys), np.mean(sums)

    def recalibrate_sum_baseline(self):
        print("\n--- Recalibrating Signal Baseline ---")
        print("Measuring current beam power...")
        _, _, new_sum = self.read_kqd_avg()
        if new_sum is None:
            raise RuntimeError("Could not get a reading. Please ensure beam is on the detector.")
        self.baseline_sum = new_sum
        print(f"New baseline sum established: {self.baseline_sum:.4f}")

    def _scan_actuator_response(self, actuator_name, move_func, get_pos_func, start_pos, step_size, max_steps, sum_threshold):
        positions, x_readings, y_readings = [], [], []
        print(f"Scanning {actuator_name.upper()} actuator...")
        for i in range(max_steps + 1):
            current_pos = start_pos + i * step_size
            move_func(current_pos)
            time.sleep(0.1) # Wait for move to settle
            
            x, y, current_sum = self.read_kqd_avg(n=5)
            if x is None or current_sum < sum_threshold:
                print(f"  - Scan stopped at step {i}: Signal lost or below threshold.", file=sys.stderr)
                break

            positions.append(current_pos)
            x_readings.append(x)
            y_readings.append(y)
            print(f"  - Step {i}: Pos={current_pos}, Reading=({x:.4f}, {y:.4f})")
        
        move_func(start_pos) # Return to start
        time.sleep(0.2)
        
        if len(positions) < 2:
            raise RuntimeError(f"Scan for {actuator_name.upper()} failed to collect enough data points.")

        slope_x = np.polyfit(positions, x_readings, 1)[0]
        slope_y = np.polyfit(positions, y_readings, 1)[0]
        return slope_x, slope_y

    def calibrate(self, tic_step_size=10, km_step_size=400, max_scan_steps=20, sum_drop_ratio=0.7):
        print("\n--- Starting Scan-Based System Calibration ---")
        if not self.kqd or not self.tic or not self.km:
            raise ConnectionError("Not all slow steering hardware is connected for calibration.")

        print("Taking initial neutral reading...")
        x0, y0, sum0 = self.read_kqd_avg()
        if x0 is None:
            raise RuntimeError("Calibration failed: Could not get an initial KQD reading.")
        
        self.baseline_sum = sum0
        scan_sum_threshold = self.baseline_sum * sum_drop_ratio
        print(f"Initial Neutral QD: ({x0:.6f}, {y0:.6f})")
        print(f"Baseline Sum: {self.baseline_sum:.4f}, Scan Stop Threshold: {scan_sum_threshold:.4f}")

        try:
            tic_start_pos = self.tic.get_current_position()
            tic_slope_x, tic_slope_y = self._scan_actuator_response(
                'tic', self.tic.set_target_position, self.tic.get_current_position,
                tic_start_pos, tic_step_size, max_scan_steps, scan_sum_threshold
            )
            print(f"Tic Influence (Slope): dX/dTic={tic_slope_x:.6f}, dY/dTic={tic_slope_y:.6f}")

            km_start_pos = self.km.get_position()
            self.km.wait_move()
            km_slope_x, km_slope_y = self._scan_actuator_response(
                'km', self.km.move_to, self.km.get_position,
                km_start_pos, km_step_size, max_scan_steps, scan_sum_threshold
            )
            print(f"KM Influence (Slope): dX/dKM={km_slope_x:.6f}, dY/dKM={km_slope_y:.6f}")

            M = np.array([[tic_slope_x, km_slope_x], [tic_slope_y, km_slope_y]])
            print("\nCalibration Matrix M (from slopes):\n", M)
            self.Minv = np.linalg.inv(M)
            print("\nInverse Calibration Matrix Minv:\n", self.Minv)
            print("Calibration successful.")

        except np.linalg.LinAlgError:
            self.Minv = None
            raise RuntimeError("Calibration failed: Matrix is singular (actuator moves had no effect).")
        except RuntimeError as e:
            self.Minv = None
            raise

    def run_slow_feedback_loop(self):
        if self.Minv is None: return "error_not_calibrated"
        if self.baseline_sum is None: return "error_no_baseline"

        sum_threshold = self.baseline_sum * 0.5
        print(f"\nStarting feedback loop. Stop threshold: {sum_threshold:.4f}")

        while not self.stop_loop_event.is_set():
            r = self.kqd.get_readings()
            if r is None:
                time.sleep(0.2)
                continue

            if r.sum < sum_threshold:
                print("\nBEAM SIGNAL LOST. Halting loop.")
                self.pid_x.reset()
                self.pid_y.reset()
                return "stopped_beam_lost"

            x_err, y_err = r.xdiff, r.ydiff
            dx = self.pid_x.compute(x_err)
            dy = self.pid_y.compute(y_err)

            d_actuator = self.Minv @ np.array([dx, dy])
            d_tic, d_gon = d_actuator[0], d_actuator[1]
            
            current_tic_pos = self.tic.get_current_position()
            self.tic.set_target_position(current_tic_pos + int(d_tic))
            self.km.move_by(int(d_gon))
            
            print(f"Err:[{x_err:.4f},{y_err:.4f}] | Sum:{r.sum:.3f} | Corr:[{d_tic:.2f},{d_gon:.2f}]", end='\r')
            time.sleep(0.1)
        
        return "stopped_by_user"

    def manual_x(self, distance, relative=True):
        if not self.tic: raise ConnectionError("Tic controller not connected.")
        if relative:
            target_pos = self.tic.get_current_position() + distance
        else:
            target_pos = distance
        self.tic.set_target_position(target_pos)
        time.sleep(0.05)

    def manual_y(self, distance, relative=True):
        if not self.km: raise ConnectionError("Kinesis motor not connected.")
        if relative:
            self.km.move_by(distance)
        else:
            self.km.move_to(distance)
        self.km.wait_move()
        
    def set_tic_soft_limits(self, min_pos, max_pos):
        if not self.tic: raise ConnectionError("Tic controller not connected.")
        self.tic.set_setting("min_target_position", int(min_pos))
        self.tic.set_setting("max_target_position", int(max_pos))

    def clear_tic_soft_limits(self):
        if not self.tic: raise ConnectionError("Tic controller not connected.")
        self.tic.set_setting("min_target_position", -2147483648)
        self.tic.set_setting("max_target_position", 2147483647)

    def set_km_soft_limits(self, min_pos, max_pos):
        if not self.km: raise ConnectionError("Kinesis motor not connected.")
        self.km.set_position_software_limits(min_pos, max_pos)

    def clear_km_soft_limits(self):
        if not self.km: raise ConnectionError("Kinesis motor not connected.")
        min_limit, max_limit = self.km.get_travel_range()
        self.km.set_position_software_limits(min_limit, max_limit)

