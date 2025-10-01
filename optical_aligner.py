import time
import numpy as np
import sys
import threading
import json

# Conditional Imports for Simulation vs. Real Hardware
try:
    from pylablib.devices import Thorlabs
    from ticlib import TicUSB
    import usb.core
except ImportError:
    print("WARNING: Real hardware libraries not found. Hardware control will fail unless in simulation mode.")

from pid_controller import PID

class OpticalAligner:
    """
    Manages hardware and control logic for an automated optical alignment system.
    """
    def __init__(self, kqd_port=None, km_port=None, simulation=False):
        self.simulation = simulation
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
        self.x_cal_results = None
        self.y_cal_results = None
        self.x_hysteresis_comp = 0
        self.y_hysteresis_comp = 0
        self.kqd_lock = threading.Lock()
        self._latest_kqd_reading = None # For producer-consumer model

    def connect_all(self):
        """Connects to all available hardware without failing on optional components."""
        print("--- Connecting to All Available Hardware ---")
        if self.simulation:
            print("--- RUNNING IN SIMULATION MODE ---")
            from mock_hardware import MockKQD, MockKinesisMotor, MockTic
            self.kqd = MockKQD(self.kqd_port)
            self.km = MockKinesisMotor(self.km_port)
            self.tic = MockTic()
        else:
            if self.kqd_port:
                self.kqd = Thorlabs.KinesisQuadDetector(self.kqd_port)
                print("KQD connected.")
            if self.km_port:
                self.km = Thorlabs.KinesisMotor(self.km_port)
                print("Kinesis Motor connected.")
                self.tic = TicUSB()
                print("Tic controller connected.")
        
        if self.tic:
            self.tic.halt_and_set_position(0); self.tic.energize(); self.tic.exit_safe_start()
            print("Tic stage initialized.")
        
        time.sleep(1.0)
        print("Hardware connection process complete.")

    def disconnect_all(self):
        print("\n--- Disconnecting All Hardware ---")
        if self.tic and not self.simulation:
            try:
                self.tic.deenergize()
            except usb.core.USBError:
                print("Could not de-energize Tic, may have already disconnected.")
        if self.km and self.km.is_opened(): self.km.close()
        if self.kqd and self.kqd.is_opened():
            with self.kqd_lock:
                self.kqd.set_operation_mode("monitor")
                self.kqd.close()
        
        self.clear_calibration_data()
        self.kqd, self.km, self.tic = None, None, None
        print("All hardware disconnected safely.")
        
    def clear_calibration_data(self):
        self.Minv = None; self.baseline_sum = None
        self.x_cal_results = None; self.y_cal_results = None
        self.x_hysteresis_comp = 0; self.y_hysteresis_comp = 0
        print("Cleared all stored calibration data.")

    def save_calibration_to_file(self, filepath):
        minv_list = self.Minv.tolist() if self.Minv is not None else None
        data = {
            "Minv": minv_list, "baseline_sum": self.baseline_sum,
            "x_cal_results": self.x_cal_results, "y_cal_results": self.y_cal_results,
            "x_hysteresis_comp": self.x_hysteresis_comp, "y_hysteresis_comp": self.y_hysteresis_comp,
        }
        with open(filepath, 'w') as f: json.dump(data, f, indent=4)
        print(f"Calibration data saved to {filepath}")

    def load_calibration_from_file(self, filepath):
        with open(filepath, 'r') as f: data = json.load(f)
        self.Minv = np.array(data["Minv"]) if data.get("Minv") is not None else None
        self.baseline_sum = data.get("baseline_sum")
        self.x_cal_results = data.get("x_cal_results")
        self.y_cal_results = data.get("y_cal_results")
        self.x_hysteresis_comp = data.get("x_hysteresis_comp", 0)
        self.y_hysteresis_comp = data.get("y_hysteresis_comp", 0)
        print(f"Calibration data loaded from {filepath}")
    
    def manage_fast_steering_mode(self):
        """Interactive CLI for fast steering modes."""
        if not self.kqd: raise ConnectionError("Fast steering hardware not connected.")
        print("\n--- Fast Steering Control ---")
        with self.kqd_lock:
            self.kqd.set_operation_mode("open_loop")
        try:
            while True:
                with self.kqd_lock:
                    current_mode = self.kqd.get_operation_mode()
                print(f"\nCurrent Mode: {current_mode.upper()}")
                print("1. Change Mode\n2. Change Settings\n3. Exit")
                choice = input("Select an option: ").strip()
                if choice == '1': self._run_selected_mode()
                elif choice == '2': self._manage_output_settings()
                elif choice == '3': break
        finally:
            with self.kqd_lock:
                self.kqd.set_operation_mode("monitor")

    def _run_selected_mode(self):
        """Helper for fast steering CLI mode changes."""
        mode_choice = input("Select mode (monitor, open_loop, closed_loop): ").lower().strip()
        if mode_choice not in ["monitor", "open_loop", "closed_loop"]:
            print("Invalid mode."); return
        try:
            with self.kqd_lock:
                if mode_choice == "closed_loop":
                    p = float(input(f"  P gain [{self.last_pid_params['p']}]: ") or self.last_pid_params['p'])
                    i = float(input(f"  I gain [{self.last_pid_params['i']}]: ") or self.last_pid_params['i'])
                    d = float(input(f"  D gain [{self.last_pid_params['d']}]: ") or self.last_pid_params['d'])
                    self.last_pid_params = {'p': p, 'i': i, 'd': d}
                    self.kqd.set_pid_parameters(p=p, i=i, d=d)
                elif mode_choice == "open_loop":
                    x_pos = float(input("  X voltage (-10 to 10): ")); y_pos = float(input("  Y voltage (-10 to 10): "))
                    self.kqd.set_manual_output(xpos=x_pos, ypos=y_pos)
                self.kqd.set_operation_mode(mode_choice)
            print(f"\nMode set to '{mode_choice}'. Press Ctrl+C to return.")
            while True:
                with self.kqd_lock:
                    r = self.kqd.get_readings()
                print(f"X_diff: {r.xdiff:.4f}, Y_diff: {r.ydiff:.4f}, Sum: {r.sum:.4f}", end='\r')
                time.sleep(0.1)
        except (KeyboardInterrupt, ValueError, Exception) as e:
            print(f"\nReturning to menu. Error: {e}", file=sys.stderr)
        
    def _manage_output_settings(self):
        """Helper for fast steering CLI settings."""
        try:
            with self.kqd_lock:
                current_params_obj = self.kqd.get_output_parameters()
            
            param_names = ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out']
            current = {name: getattr(current_params_obj, name) for name in param_names}
            print("\n--- Current Output Settings ---")
            for key, value in current.items(): print(f"  - {key}: {value}")
            if input("\nUpdate settings? (y/n): ").lower() != 'y': return
            print("\nEnter new value or press Enter to keep current setting.")
            new_params = {}
            for key, value in current.items():
                if isinstance(value, (int, float)):
                    new_params[key] = float(input(f"  {key} [{value}]: ") or value)
                else:
                    new_params[key] = input(f"  {key} [{value}]: ") or value
            with self.kqd_lock:
                self.kqd.set_output_parameters(**new_params)
            print("\nSettings updated successfully.")
        except Exception as e:
            print(f"\nError managing settings: {e}", file=sys.stderr)
        
    def get_fast_steering_params(self):
        if not self.kqd: raise ConnectionError("Fast steering hardware not connected.")
        with self.kqd_lock:
            pid_obj = self.kqd.get_pid_parameters()
            params_obj = self.kqd.get_output_parameters()
        pid_dict = {'p': pid_obj.p, 'i': pid_obj.i, 'd': pid_obj.d}
        param_names = ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out']
        params_dict = {name: getattr(params_obj, name) for name in param_names}
        return (pid_dict, params_dict)
        
    def set_fast_steering_params(self, pid, params):
        if not self.kqd: raise ConnectionError("Fast steering hardware not connected.")
        with self.kqd_lock:
            self.kqd.set_pid_parameters(p=pid['p'], i=pid['i'], d=pid['d'])
            self.kqd.set_output_parameters(
                xgain=params['xgain'], ygain=params['ygain'],
                xmin=params['xmin'], xmax=params['xmax'],
                ymin=params['ymin'], ymax=params['ymax']
            )

    def get_pid_settings(self):
        return {'x': {'p':self.pid_x.Kp, 'i':self.pid_x.Ki, 'd':self.pid_x.Kd}, 'y': {'p':self.pid_y.Kp, 'i':self.pid_y.Ki, 'd':self.pid_y.Kd}}

    def set_pid_settings(self, pid_x_p, pid_x_i, pid_x_d, pid_y_p, pid_y_i, pid_y_d):
        self.pid_x.Kp, self.pid_x.Ki, self.pid_x.Kd = pid_x_p, pid_x_i, pid_x_d
        self.pid_y.Kp, self.pid_y.Ki, self.pid_y.Kd = pid_y_p, pid_y_i, pid_y_d

    def get_tic_settings(self):
        if not self.tic: raise ConnectionError("Tic not connected.")
        return {'velocity': self.tic.get_max_speed(), 'current_limit': self.tic.get_current_limit(), 'step_mode': self.tic.get_step_mode()}

    def set_tic_settings(self, velocity, current_limit, step_mode):
        if not self.tic: raise ConnectionError("Tic not connected.")
        self.tic.set_max_speed(velocity); self.tic.set_current_limit(current_limit); self.tic.set_step_mode(step_mode)

    def get_km_settings(self):
        if not self.km: raise ConnectionError("Kinesis Motor not connected.")
        return {'velocity': self.km.get_velocity_parameters().max_velocity}

    def set_km_settings(self, velocity):
        if not self.km: raise ConnectionError("Kinesis Motor not connected.")
        current_accel = self.km.get_velocity_parameters().acceleration
        self.km.setup_velocity(max_velocity=velocity, acceleration=current_accel)

    def read_kqd_avg(self, n=20, delay=0.05):
        if not self.kqd: return None, None, None
        xs, ys, sums = [], [], []
        for _ in range(n):
            try:
                with self.kqd_lock:
                    r = self.kqd.get_readings()
                if r: xs.append(r.xdiff); ys.append(r.ydiff); sums.append(r.sum)
                time.sleep(delay)
            except Exception: pass
        if not xs: return None, None, None
        return np.mean(xs), np.mean(ys), np.mean(sums)

    def get_latest_reading(self):
        return self._latest_kqd_reading

    def recalibrate_sum_baseline(self):
        print("\nRecalibrating Signal Baseline...")
        _, _, new_sum = self.read_kqd_avg()
        if new_sum is None: raise RuntimeError("Could not get a reading from KQD.")
        self.baseline_sum = new_sum
        print(f"New baseline sum: {self.baseline_sum:.4f}")

    def _scan_actuator_response_roundtrip(self, actuator_name, move_func, start_pos, step_size, max_steps, sum_threshold):
        print(f"Scanning {actuator_name.upper()}..."); forward_positions, forward_x, forward_y = [], [], []
        print("  - Forward direction...")
        for i in range(max_steps + 1):
            pos = start_pos + i * step_size; move_func(pos)
            if actuator_name == 'km': self.km.wait_move()
            time.sleep(0.05)
            x, y, s = self.read_kqd_avg(n=5)
            if x is None or s < sum_threshold: print(f"    - Scan stopped at step {i}: Signal lost."); break
            forward_positions.append(pos); forward_x.append(x); forward_y.append(y)
        backward_positions, backward_x, backward_y = [], [], []
        print("  - Backward direction...")
        last_forward_pos = forward_positions[-1]
        for i in range(len(forward_positions)):
            pos = last_forward_pos - i * step_size; move_func(pos)
            if actuator_name == 'km': self.km.wait_move()
            time.sleep(0.05)
            x, y, s = self.read_kqd_avg(n=5)
            if x is None: continue
            backward_positions.append(pos); backward_x.append(x); backward_y.append(y)
        if len(forward_positions) < 2: raise RuntimeError(f"Scan for {actuator_name.upper()} failed.")
        slope_x = np.polyfit(forward_positions, forward_x, 1)[0]; slope_y = np.polyfit(forward_positions, forward_y, 1)[0]
        interp_x = np.interp(forward_positions, backward_positions[::-1], backward_x[::-1])
        interp_y = np.interp(forward_positions, backward_positions[::-1], backward_y[::-1])
        hyst_offset_x = np.mean(np.array(forward_x) - interp_x); hyst_offset_y = np.mean(np.array(forward_y) - interp_y)
        hyst_comp_x = hyst_offset_x / slope_x if abs(slope_x) > 1e-9 else 0
        hyst_comp_y = hyst_offset_y / slope_y if abs(slope_y) > 1e-9 else 0
        primary_hysteresis_comp = hyst_comp_x if abs(slope_x) > abs(slope_y) else hyst_comp_y
        return slope_x, slope_y, primary_hysteresis_comp

    def _calibrate_axis(self, axis_name, step_size, max_scan_steps=20, sum_drop_ratio=0.7):
        print(f"\n--- Starting {axis_name.upper()}-Axis Calibration ---")
        if not all([self.kqd, self.tic, self.km]): raise ConnectionError("Not all hardware connected.")
        _, _, sum0 = self.read_kqd_avg()
        if sum0 is None: raise RuntimeError("Could not get an initial KQD reading.")
        self.baseline_sum = sum0; scan_sum_threshold = self.baseline_sum * sum_drop_ratio
        print(f"Using Baseline Sum: {self.baseline_sum:.4f}, Scan Stop Threshold: {scan_sum_threshold:.4f}")
        if axis_name == 'x':
            start_pos = self.tic.get_current_position()
            slope_x, slope_y, hyst_comp = self._scan_actuator_response_roundtrip('tic', self.tic.set_target_position, start_pos, step_size, max_scan_steps, scan_sum_threshold)
            self.x_cal_results = {'slope_x': slope_x, 'slope_y': slope_y}; self.x_hysteresis_comp = hyst_comp
            print(f"X-Axis Hysteresis Comp: {hyst_comp:.2f} steps")
        elif axis_name == 'y':
            start_pos = self.km.get_position()
            slope_x, slope_y, hyst_comp = self._scan_actuator_response_roundtrip('km', self.km.move_to, start_pos, step_size, max_scan_steps, scan_sum_threshold)
            self.y_cal_results = {'slope_x': slope_x, 'slope_y': slope_y}; self.y_hysteresis_comp = hyst_comp
            print(f"Y-Axis Hysteresis Comp: {hyst_comp:.2f} steps")
        self._calculate_final_matrix()

    def calibrate_x_axis(self, step_size=10): self._calibrate_axis('x', step_size=step_size)
    def calibrate_y_axis(self, step_size=400): self._calibrate_axis('y', step_size=step_size)

    def _calculate_final_matrix(self):
        if self.x_cal_results and self.y_cal_results:
            print("\n--- Both Axes Calibrated: Calculating Final Matrix ---")
            M = np.array([[self.x_cal_results['slope_x'], self.y_cal_results['slope_x']], [self.x_cal_results['slope_y'], self.y_cal_results['slope_y']]])
            print("Calibration Matrix M:\n", M)
            try:
                self.Minv = np.linalg.inv(M)
                print("\nInverse Calibration Matrix Minv:\n", self.Minv)
                print("System is fully calibrated.")
            except np.linalg.LinAlgError:
                self.Minv = None; print("\nError: Final matrix is singular.", file=sys.stderr)

    def run_slow_feedback_loop(self, loop_gain_x=1.0, loop_gain_y=1.0, status_callback=None):
        if self.Minv is None: return "error_not_calibrated"
        if self.baseline_sum is None: return "error_no_baseline"
        print(f"\nStarting feedback loop with gains X={loop_gain_x}, Y={loop_gain_y}.")
        while not self.stop_loop_event.is_set():
            sum_threshold = self.baseline_sum * 0.5
            with self.kqd_lock:
                r = self.kqd.get_readings()
                self._latest_kqd_reading = r
            if r is None: time.sleep(0.2); continue
            if r.sum < sum_threshold:
                print(f"\nBEAM SIGNAL LOST (Sum: {r.sum:.4f}).")
                self.tic.halt_and_hold(); self.km.stop()
                self.pid_x.reset(); self.pid_y.reset()
                return "stopped_beam_lost"
            x_err, y_err = r.xdiff, r.ydiff
            dx, dy = self.pid_x.compute(x_err), self.pid_y.compute(y_err)
            d_actuator = self.Minv @ np.array([dx, dy])
            final_d_tic, final_d_gon = d_actuator[0] * loop_gain_x, d_actuator[1] * loop_gain_y
            current_tic_pos = self.tic.get_current_position()
            self.tic.set_target_position(current_tic_pos + int(final_d_tic))
            self.km.move_by(int(final_d_gon))
            if status_callback: status_callback(x_err, y_err, r.sum, final_d_tic, final_d_gon)
            time.sleep(0.1)
        print("\nLoop stopped by user. Resetting PIDs.")
        self.pid_x.reset(); self.pid_y.reset()
        return "stopped_by_user"

    def manual_x(self, distance, relative=True):
        if not self.tic: raise ConnectionError("Tic not connected.")
        if relative: target_pos = self.tic.get_current_position() + distance
        else: target_pos = distance
        self.tic.set_target_position(target_pos)
        time.sleep(0.05)

    def manual_y(self, distance, relative=True):
        if not self.km: raise ConnectionError("Kinesis motor not connected.")
        if relative:
            target_pos = self.km.get_position() + distance
            self.km.move_to(target_pos)
        else:
            self.km.move_to(distance)
        self.km.wait_move()

