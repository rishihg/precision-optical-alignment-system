import time
import numpy as np
import sys
import threading
import json
from collections import deque

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
    Supports simultaneous fast and slow steering with efficient multithreading.
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
        
        # Threading components
        self.kqd_lock = threading.Lock()
        self.km_lock = threading.Lock()
        self._stop_slow_loop_event = threading.Event()
        self._stop_fast_loop_event = threading.Event()
        self.stop_kqd_reader = threading.Event()
        
        # KQD reading buffer (producer-consumer model)
        self._kqd_buffer = deque(maxlen=10)
        self._kqd_buffer_lock = threading.Lock()
        self._kqd_reader_thread = None
        self._latest_kqd_reading = None
        
        # Thread references
        self._slow_loop_thread = None
        self._fast_loop_thread = None
        
        # Calibration data
        self.x_cal_results = None
        self.y_cal_results = None
        self.x_hysteresis_comp = 0
        self.y_hysteresis_comp = 0
        
        # Status tracking
        self.slow_loop_status = "stopped"
        self.fast_loop_status = "stopped"
        self._status_lock = threading.Lock()

        # Command tracking for display
        self._last_slow_commands = {'tic_cmd': 0, 'km_cmd': 0}
        self._slow_cmd_lock = threading.Lock()



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
            if self.km_port:  # Only connect Tic if KM is available
                self.tic = TicUSB()
                print("Tic controller connected.")
        
        if self.tic:
            self.tic.halt_and_set_position(0)
            self.tic.energize()
            self.tic.exit_safe_start()
            print("Tic stage initialized.")
        
        time.sleep(1.0)
        print("Hardware connection process complete.")
        
        # Start KQD reader thread if KQD is available
        if self.kqd:
            self._start_kqd_reader()

    def disconnect_all(self):
        """Safely disconnect all hardware and stop all threads."""
        print("\n--- Disconnecting All Hardware ---")
        
        # Stop all control loops first
        self.stop_all_loops()
        
        # Stop KQD reader
        if self._kqd_reader_thread and self._kqd_reader_thread.is_alive():
            self.stop_kqd_reader.set()
            self._kqd_reader_thread.join(timeout=2.0)
        
        # Disconnect hardware
        if self.tic and not self.simulation:
            try:
                self.tic.deenergize()
            except usb.core.USBError:
                print("Could not de-energize Tic, may have already disconnected.")
        
        if self.km and self.km.is_opened():
            self.km.close()
        
        if self.kqd and self.kqd.is_opened():
            with self.kqd_lock:
                self.kqd.set_operation_mode("open_loop")
                self.kqd.close()

    def _scan_actuator_response_roundtrip(self, actuator_name, move_func, start_pos, 
                                         step_size, max_steps, sum_threshold):
        """Scan actuator response in both directions for calibration."""
        print(f"Scanning {actuator_name.upper()}...")
        forward_positions, forward_x, forward_y = [], [], []
        
        print("  - Forward direction...")
        for i in range(max_steps + 1):
            pos = start_pos + i * step_size
            if actuator_name == 'km':
                with self.km_lock:
                    move_func(pos)
                    self.km.wait_move()
            else:
                move_func(pos)
            time.sleep(0.05)
            
            r = self.get_averaged_reading(n=5)
            if r is None or r.sum < sum_threshold:
                print(f"    - Scan stopped at step {i}: Signal lost.")
                break
            
            forward_positions.append(pos)
            forward_x.append(r.xdiff)
            forward_y.append(r.ydiff)
        
        backward_positions, backward_x, backward_y = [], [], []
        print("  - Backward direction...")
        last_forward_pos = forward_positions[-1]
        
        for i in range(len(forward_positions)):
            pos = last_forward_pos - i * step_size
            if actuator_name == 'km':
                with self.km_lock:
                    move_func(pos)
                    self.km.wait_move()
            else:
                move_func(pos)
            time.sleep(0.05)
            
            r = self.get_averaged_reading(n=5)
            if r is None:
                print(f"    - WARNING: No reading at step {i}, skipping this point.")
                continue
            
            backward_positions.append(pos)
            backward_x.append(r.xdiff)
            backward_y.append(r.ydiff)
        
        # Print completion status
        print(f"    - Backward scan completed {len(backward_positions)} steps.")
        
        if len(forward_positions) < 2:
            raise RuntimeError(f"Scan for {actuator_name.upper()} failed.")
        
        # Calculate slopes and hysteresis
        slope_x = np.polyfit(forward_positions, forward_x, 1)[0]
        slope_y = np.polyfit(forward_positions, forward_y, 1)[0]
        
        interp_x = np.interp(forward_positions, backward_positions[::-1], backward_x[::-1])
        interp_y = np.interp(forward_positions, backward_positions[::-1], backward_y[::-1])
        
        hyst_offset_x = np.mean(np.array(forward_x) - interp_x)
        hyst_offset_y = np.mean(np.array(forward_y) - interp_y)
        
        hyst_comp_x = hyst_offset_x / slope_x if abs(slope_x) > 1e-9 else 0
        hyst_comp_y = hyst_offset_y / slope_y if abs(slope_y) > 1e-9 else 0
        
        primary_hysteresis_comp = hyst_comp_x if abs(slope_x) > abs(slope_y) else hyst_comp_y
        
        return slope_x, slope_y, primary_hysteresis_comp

    def _calibrate_axis(self, axis_name, step_size, max_scan_steps=20, sum_drop_ratio=0.7):
        """Calibrate a single axis (x or y)."""
        print(f"\n--- Starting {axis_name.upper()}-Axis Calibration ---")
        
        if not all([self.kqd, self.tic, self.km]):
            raise ConnectionError("Not all hardware connected.")
        
        # Get initial baseline
        time.sleep(0.2)  # Allow buffer to fill
        r = self.get_averaged_reading(n=20)
        if r is None:
            raise RuntimeError("Could not get an initial KQD reading.")
        
        self.baseline_sum = r.sum
        scan_sum_threshold = self.baseline_sum * sum_drop_ratio
        print(f"Using Baseline Sum: {self.baseline_sum:.4f}, Scan Stop Threshold: {scan_sum_threshold:.4f}")
        
        if axis_name == 'x':
            start_pos = self.tic.get_current_position()
            slope_x, slope_y, hyst_comp = self._scan_actuator_response_roundtrip(
                'tic', self.tic.set_target_position, start_pos, 
                step_size, max_scan_steps, scan_sum_threshold
            )
            self.x_cal_results = {'slope_x': slope_x, 'slope_y': slope_y}
            self.x_hysteresis_comp = hyst_comp
            print(f"X-Axis Hysteresis Comp: {hyst_comp:.2f} steps")
            
        elif axis_name == 'y':
            start_pos = self.km.get_position()
            slope_x, slope_y, hyst_comp = self._scan_actuator_response_roundtrip(
                'km', self.km.move_to, start_pos, 
                step_size, max_scan_steps, scan_sum_threshold
            )
            self.y_cal_results = {'slope_x': slope_x, 'slope_y': slope_y}
            self.y_hysteresis_comp = hyst_comp
            print(f"Y-Axis Hysteresis Comp: {hyst_comp:.2f} steps")
        
        self._calculate_final_matrix()

    def calibrate_x_axis(self, step_size=10, max_scan_steps=20, sum_drop_ratio=0.7):
        """Calibrate X-axis."""
        self._calibrate_axis('x', step_size=step_size)

    def calibrate_y_axis(self, step_size=400, max_scan_steps=20, sum_drop_ratio=0.7):
        """Calibrate Y-axis."""
        self._calibrate_axis('y', step_size=step_size)

    def _calculate_final_matrix(self):
        """Calculate final calibration matrix from axis calibrations."""
        if self.x_cal_results and self.y_cal_results:
            print("\n--- Both Axes Calibrated: Calculating Final Matrix ---")
            M = np.array([
                [self.x_cal_results['slope_x'], self.y_cal_results['slope_x']],
                [self.x_cal_results['slope_y'], self.y_cal_results['slope_y']]
            ])
            print("Calibration Matrix M:\n", M)
            
            try:
                self.Minv = np.linalg.inv(M)
                print("\nInverse Calibration Matrix Minv:\n", self.Minv)
                print("System is fully calibrated.")
            except np.linalg.LinAlgError:
                self.Minv = None
                print("\nError: Final matrix is singular.", file=sys.stderr)

    def get_recommended_gains(self):
        """
        Calculate recommended gain signs based on calibration matrix.
        Returns dict with recommended signs for X and Y loop gains.
        """
        if self.Minv is None:
            return None
        
        # For negative feedback (error reduction):
        # If Minv[i,i] > 0: need negative gain (error positive → move negative)
        # If Minv[i,i] < 0: need positive gain (error positive → move positive)
        
        recommended = {
            'x_gain_sign': -1 if self.Minv[0, 0] > 0 else 1,
            'y_gain_sign': -1 if self.Minv[1, 1] > 0 else 1,
            'x_coupling': abs(self.Minv[0, 1]) / abs(self.Minv[0, 0]) if abs(self.Minv[0, 0]) > 1e-9 else 0,
            'y_coupling': abs(self.Minv[1, 0]) / abs(self.Minv[1, 1]) if abs(self.Minv[1, 1]) > 1e-9 else 0
        }
        
        return recommended   
    
    def get_last_slow_commands(self):
        """Get the last commands sent to actuators in slow loop."""
        with self._slow_cmd_lock:
            return self._last_slow_commands.copy()
        

    # ==================== KQD READER THREAD ====================
    
    def _start_kqd_reader(self):
        """Start background thread to continuously read KQD data."""
        if self._kqd_reader_thread and self._kqd_reader_thread.is_alive():
            print("KQD reader already running.")
            return
        
        self.stop_kqd_reader.clear()
        self._kqd_reader_thread = threading.Thread(
            target=self._kqd_reader_loop,
            name="KQD-Reader",
            daemon=True
        )
        self._kqd_reader_thread.start()
        print("KQD reader thread started.")
    
    def _kqd_reader_loop(self):
        """Continuously read KQD and populate buffer."""
        if not self.kqd:
            return
        
        while not self.stop_kqd_reader.is_set():
            try:
                with self.kqd_lock:
                    reading = self.kqd.get_readings()
                
                if reading:
                    with self._kqd_buffer_lock:
                        self._kqd_buffer.append(reading)
                        self._latest_kqd_reading = reading
                
                time.sleep(0.01)  # 100 Hz reading rate
                
            except Exception as e:
                print(f"KQD reader error: {e}", file=sys.stderr)
                time.sleep(0.1)
    
    def get_latest_reading(self):
        """Get the most recent KQD reading (non-blocking)."""
        with self._kqd_buffer_lock:
            return self._latest_kqd_reading
    
    def get_averaged_reading(self, n=5):
        """Get averaged reading from recent buffer."""
        with self._kqd_buffer_lock:
            if len(self._kqd_buffer) == 0:
                return None
            
            # Get up to n most recent readings
            recent = list(self._kqd_buffer)[-n:]
            
            x_vals = [r.xdiff for r in recent]
            y_vals = [r.ydiff for r in recent]
            sum_vals = [r.sum for r in recent]
            
            # Create averaged reading object
            class AvgReading:
                def __init__(self, xdiff, ydiff, sum_val):
                    self.xdiff = xdiff
                    self.ydiff = ydiff
                    self.sum = sum_val
            
            return AvgReading(
                np.mean(x_vals),
                np.mean(y_vals),
                np.mean(sum_vals)
            )
        
    def get_current_output(self):
        """Get current output voltages being sent to piezo actuators."""
        if not self.kqd:
            return None
        
        try:
            with self.kqd_lock:
                output = self.kqd.get_manual_output()
                
                # Named tuple is iterable, so we can index it
                if isinstance(output, (tuple, list)) and len(output) >= 2:
                    return {'xout': output[0], 'yout': output[1]}
                # Or use attributes directly
                elif hasattr(output, 'xpos') and hasattr(output, 'ypos'):
                    return {'xout': output.xpos, 'yout': output.ypos}
                else:
                    return None
                
        except Exception as e:
            print(f"Error getting output: {e}")
            return None
        
        

    # ==================== SLOW STEERING CONTROL ====================
    
    def run_slow_feedback_loop(self, loop_gain_x=1.0, loop_gain_y=1.0, 
                               status_callback=None, threaded=False):
        """
        Run slow steering feedback loop.
        
        Args:
            loop_gain_x: X-axis loop gain multiplier
            loop_gain_y: Y-axis loop gain multiplier
            status_callback: Optional callback function(x_err, y_err, sum, dx, dy)
            threaded: If True, run in background thread
        """
        if threaded:
            return self._start_slow_loop_thread(loop_gain_x, loop_gain_y, status_callback)
        else:
            return self._slow_feedback_loop_impl(loop_gain_x, loop_gain_y, status_callback)
    
    def _start_slow_loop_thread(self, loop_gain_x, loop_gain_y, status_callback):
        """Start slow loop in background thread."""
        if self._slow_loop_thread and self._slow_loop_thread.is_alive():
            return "error_already_running"
        
        self._stop_slow_loop_event.clear()
        self._slow_loop_thread = threading.Thread(
            target=self._slow_feedback_loop_impl,
            args=(loop_gain_x, loop_gain_y, status_callback),
            name="Slow-Feedback",
            daemon=True
        )
        self._slow_loop_thread.start()
        
        with self._status_lock:
            self.slow_loop_status = "running"
        
        return "started"
    
    def _slow_feedback_loop_impl(self, loop_gain_x, loop_gain_y, status_callback):
        """Internal implementation of slow feedback loop."""
        if self.Minv is None:
            with self._status_lock:
                self.slow_loop_status = "error_not_calibrated"
            return "error_not_calibrated"
        
        if self.baseline_sum is None:
            with self._status_lock:
                self.slow_loop_status = "error_no_baseline"
            return "error_no_baseline"
        
        if not (self.tic and self.km):
            with self._status_lock:
                self.slow_loop_status = "error_hardware_missing"
            return "error_hardware_missing"
        
        print(f"\nStarting slow feedback loop with gains X={loop_gain_x}, Y={loop_gain_y}.")
        self.pid_x.reset()
        self.pid_y.reset()
        
        sum_threshold = self.baseline_sum * 0.5
        
        while not self._stop_slow_loop_event.is_set():
            try:
                # Get reading from buffer (non-blocking)
                r = self.get_latest_reading()
                
                if r is None:
                    time.sleep(0.1)
                    continue
                
                # Check beam signal
                if r.sum < sum_threshold:
                    print(f"\nSLOW LOOP: BEAM SIGNAL LOST (Sum: {r.sum:.4f}).")
                    self.tic.halt_and_hold()
                    self.km.stop()
                    self.pid_x.reset()
                    self.pid_y.reset()
                    with self._status_lock:
                        self.slow_loop_status = "stopped_beam_lost"
                    return "stopped_beam_lost"
                
                # Compute PID corrections
                x_err, y_err = r.xdiff, r.ydiff
                dx = self.pid_x.compute(x_err)
                dy = self.pid_y.compute(y_err)
                
                # Transform to actuator space
                d_actuator = self.Minv @ np.array([dx, dy])
                final_d_tic = d_actuator[0] * loop_gain_x
                final_d_gon = d_actuator[1] * loop_gain_y
                
                # Apply corrections
                current_tic_pos = self.tic.get_current_position()
                tic_command = int(final_d_tic)
                km_command = int(final_d_gon)

                self.tic.set_target_position(current_tic_pos + tic_command)

                with self.km_lock:
                    self.km.move_by(km_command)

                # Store commands for display
                with self._slow_cmd_lock:
                    self._last_slow_commands = {
                        'tic_cmd': tic_command,
                        'km_cmd': km_command
                    }

                
                # Status callback
                if status_callback:
                    status_callback(x_err, y_err, r.sum, final_d_tic, final_d_gon)
                
                time.sleep(0.5)  # 2 Hz update rate
                
            except Exception as e:
                print(f"\nSlow loop error: {e}", file=sys.stderr)
                time.sleep(0.1)
        
        print("\nSlow loop stopped by user. Resetting PIDs.")
        self.pid_x.reset()
        self.pid_y.reset()
        
        with self._status_lock:
            self.slow_loop_status = "stopped_by_user"
        
        return "stopped_by_user"
    
    def stop_slow_loop(self):
        """Stop the slow feedback loop."""
        if self._slow_loop_thread and self._slow_loop_thread.is_alive():
            print("Stopping slow feedback loop...")
            self._stop_slow_loop_event.set()
            self._slow_loop_thread.join(timeout=2.0)
            if self.tic:
                self.tic.halt_and_hold()
            if self.km:
                self.km.stop()
            print("Slow loop stopped.")
        
        with self._status_lock:
            self.slow_loop_status = "stopped"

    # ==================== FAST STEERING CONTROL ====================
    
    def run_fast_feedback_loop(self, mode="closed_loop", pid_params=None, 
                               manual_output=None, threaded=False, 
                               status_callback=None):
        """
        Run fast steering feedback loop.
        
        Args:
            mode: 'monitor', 'open_loop', or 'closed_loop'
            pid_params: Dict with 'p', 'i', 'd' keys (for closed_loop)
            manual_output: Dict with 'xpos', 'ypos' keys (for open_loop)
            threaded: If True, run in background thread
            status_callback: Optional callback function(xdiff, ydiff, sum)
        """
        if threaded:
            return self._start_fast_loop_thread(mode, pid_params, manual_output, status_callback)
        else:
            return self._fast_feedback_loop_impl(mode, pid_params, manual_output, status_callback)
    
    def _start_fast_loop_thread(self, mode, pid_params, manual_output, status_callback):
        """Start fast loop in background thread."""
        if self._fast_loop_thread and self._fast_loop_thread.is_alive():
            return "error_already_running"
        
        self._stop_fast_loop_event.clear()
        self._fast_loop_thread = threading.Thread(
            target=self._fast_feedback_loop_impl,
            args=(mode, pid_params, manual_output, status_callback),
            name="Fast-Feedback",
            daemon=True
        )
        self._fast_loop_thread.start()
        
        with self._status_lock:
            self.fast_loop_status = "running"
        
        return "started"
    
    def _fast_feedback_loop_impl(self, mode, pid_params, manual_output, status_callback):
        """Internal implementation of fast feedback loop."""
        if not self.kqd:
            with self._status_lock:
                self.fast_loop_status = "error_hardware_missing"
            return "error_hardware_missing"
        
        print(f"\nStarting fast feedback loop in {mode} mode.")
        
        try:
            # Set up the mode
            with self.kqd_lock:
                if mode == "closed_loop":
                    if pid_params:
                        self.kqd.set_pid_parameters(
                            p=pid_params.get('p', 1.0),
                            i=pid_params.get('i', 0.0),
                            d=pid_params.get('d', 0.0)
                        )
                        self.last_pid_params = pid_params
                elif mode == "open_loop":
                    if manual_output:
                        self.kqd.set_manual_output(
                            xpos=manual_output.get('xpos', 0.0),
                            ypos=manual_output.get('ypos', 0.0)
                        )
                
                self.kqd.set_operation_mode(mode)
            
            # Monitor loop
            while not self._stop_fast_loop_event.is_set():
                r = self.get_latest_reading()
                
                if r and status_callback:
                    status_callback(r.xdiff, r.ydiff, r.sum)
                
                time.sleep(0.05)  # 20 Hz update rate for monitoring
            
            # Clean up - return to open_loop mode
            with self.kqd_lock:
                self.kqd.set_operation_mode("open_loop")
            
            print("\nFast loop stopped.")
            
            with self._status_lock:
                self.fast_loop_status = "stopped_by_user"
            
            return "stopped_by_user"
            
        except Exception as e:
            print(f"\nFast loop error: {e}", file=sys.stderr)
            with self._status_lock:
                self.fast_loop_status = "error"
            return "error"
    
    def stop_fast_loop(self):
        """Stop the fast feedback loop."""
        if self._fast_loop_thread and self._fast_loop_thread.is_alive():
            print("Stopping fast feedback loop...")
            self._stop_fast_loop_event.set()
            self._fast_loop_thread.join(timeout=2.0)
            
            # Verify thread actually stopped
            if self._fast_loop_thread.is_alive():
                print("WARNING: Fast loop thread did not stop cleanly")
            
            # Return to open_loop mode
            if self.kqd:
                with self.kqd_lock:
                    self.kqd.set_operation_mode("open_loop")
            
            print("Fast loop stopped.")
        
        # Always update status and clear thread reference
        with self._status_lock:
            self.fast_loop_status = "stopped"
        self._fast_loop_thread = None
    
    def stop_all_loops(self):
        """Stop all running feedback loops."""
        print("\nStopping all feedback loops...")
        self.stop_slow_loop()
        self.stop_fast_loop()
        print("All loops stopped.")
    
    def get_loop_status(self):
        """Get current status of all loops."""
        with self._status_lock:
            return {
                'slow': self.slow_loop_status,
                'fast': self.fast_loop_status
            }

    # ==================== LEGACY CLI INTERFACE ====================
    
    def get_hardware_status(self):
        """Get status of all connected hardware."""
        status = {
            'kqd': {'connected': self.kqd is not None},
            'tic': {'connected': False, 'energized': False, 'position': 0},
            'km': {'connected': False, 'position': 0}
        }
        
        # Check Tic status
        if self.tic:
            try:
                status['tic']['connected'] = True
                status['tic']['position'] = self.tic.get_current_position()
                
                try:
                    variables = self.tic.get_variables()
                    operation_state = variables[0]
                    status['tic']['energized'] = bool(operation_state & 0x01)
                except:
                    status['tic']['energized'] = True
                    
            except Exception as e:
                print(f"Error reading Tic status: {e}")
        
        # Check KM status - NON-BLOCKING
        if self.km:
            if not hasattr(self, 'km_lock'):
                self.km_lock = threading.Lock()
            
            if self.km_lock.acquire(blocking=False):
                try:
                    status['km']['connected'] = True
                    status['km']['position'] = self.km.get_position()
                except Exception as e:
                    pass  # Don't print - normal during moves
                finally:
                    self.km_lock.release()
            # If we can't get lock, KM is busy - that's OK, keep old status
        
        return status

    def manage_fast_steering_mode(self):
        """Interactive CLI for fast steering modes (legacy interface)."""
        if not self.kqd:
            raise ConnectionError("Fast steering hardware not connected.")
        
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
                
                if choice == '1':
                    self._run_selected_mode()
                elif choice == '2':
                    self._manage_output_settings()
                elif choice == '3':
                    break
        finally:
            with self.kqd_lock:
                self.kqd.set_operation_mode("open_loop")

    def _run_selected_mode(self):
        """Helper for fast steering CLI mode changes."""
        mode_choice = input("Select mode (monitor, open_loop, closed_loop): ").lower().strip()
        if mode_choice not in ["monitor", "open_loop", "closed_loop"]:
            print("Invalid mode.")
            return
        
        try:
            with self.kqd_lock:
                if mode_choice == "closed_loop":
                    p = float(input(f"  P gain [{self.last_pid_params['p']}]: ") or self.last_pid_params['p'])
                    i = float(input(f"  I gain [{self.last_pid_params['i']}]: ") or self.last_pid_params['i'])
                    d = float(input(f"  D gain [{self.last_pid_params['d']}]: ") or self.last_pid_params['d'])
                    self.last_pid_params = {'p': p, 'i': i, 'd': d}
                    self.kqd.set_pid_parameters(p=p, i=i, d=d)
                elif mode_choice == "open_loop":
                    x_pos = float(input("  X voltage (-10 to 10): "))
                    y_pos = float(input("  Y voltage (-10 to 10): "))
                    self.kqd.set_manual_output(xpos=x_pos, ypos=y_pos)
                
                self.kqd.set_operation_mode(mode_choice)
            
            print(f"\nMode set to '{mode_choice}'. Press Ctrl+C to return.")
            while True:
                r = self.get_latest_reading()
                if r:
                    print(f"X_diff: {r.xdiff:.4f}, Y_diff: {r.ydiff:.4f}, Sum: {r.sum:.4f}", end='\r')
                time.sleep(0.1)
                
        except (KeyboardInterrupt, ValueError, Exception) as e:
            print(f"\nReturning to menu. Error: {e}", file=sys.stderr)

    # ==================== CALIBRATION & SETTINGS ====================
    
    def clear_calibration_data(self):
        """Clear all stored calibration data."""
        self.Minv = None
        self.baseline_sum = None
        self.x_cal_results = None
        self.y_cal_results = None
        self.x_hysteresis_comp = 0
        self.y_hysteresis_comp = 0
        print("Cleared all stored calibration data.")

    def save_calibration_to_file(self, filepath):
        """Save calibration data to JSON file."""
        minv_list = self.Minv.tolist() if self.Minv is not None else None
        data = {
            "Minv": minv_list,
            "baseline_sum": self.baseline_sum,
            "x_cal_results": self.x_cal_results,
            "y_cal_results": self.y_cal_results,
            "x_hysteresis_comp": self.x_hysteresis_comp,
            "y_hysteresis_comp": self.y_hysteresis_comp,
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Calibration data saved to {filepath}")

    def load_calibration_from_file(self, filepath):
        """Load calibration data from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        self.Minv = np.array(data["Minv"]) if data.get("Minv") is not None else None
        self.baseline_sum = data.get("baseline_sum")
        self.x_cal_results = data.get("x_cal_results")
        self.y_cal_results = data.get("y_cal_results")
        self.x_hysteresis_comp = data.get("x_hysteresis_comp", 0)
        self.y_hysteresis_comp = data.get("y_hysteresis_comp", 0)
        print(f"Calibration data loaded from {filepath}")

    def recalibrate_sum_baseline(self):
        """Recalibrate the signal baseline sum."""
        print("\nRecalibrating Signal Baseline...")
        time.sleep(0.2)  # Wait for buffer to fill
        r = self.get_averaged_reading(n=20)
        if r is None:
            raise RuntimeError("Could not get a reading from KQD.")
        self.baseline_sum = r.sum
        print(f"New baseline sum: {self.baseline_sum:.4f}")

    def read_kqd_avg(self, n=20, delay=0.05):
        """Legacy method - get averaged reading."""
        r = self.get_averaged_reading(n=n)
        if r:
            return r.xdiff, r.ydiff, r.sum
        return None, None, None

    # ==================== PID & HARDWARE SETTINGS ====================
    
    def get_pid_settings(self):
        """Get current PID settings for slow loop."""
        return {
            'x': {'p': self.pid_x.Kp, 'i': self.pid_x.Ki, 'd': self.pid_x.Kd},
            'y': {'p': self.pid_y.Kp, 'i': self.pid_y.Ki, 'd': self.pid_y.Kd}
        }

    def set_pid_settings(self, pid_x_p, pid_x_i, pid_x_d, pid_y_p, pid_y_i, pid_y_d):
        """Set PID settings for slow loop."""
        self.pid_x.Kp, self.pid_x.Ki, self.pid_x.Kd = pid_x_p, pid_x_i, pid_x_d
        self.pid_y.Kp, self.pid_y.Ki, self.pid_y.Kd = pid_y_p, pid_y_i, pid_y_d

    def get_fast_steering_params(self):
        """Get fast steering PID and output parameters."""
        if not self.kqd:
            raise ConnectionError("Fast steering hardware not connected.")
        with self.kqd_lock:
            pid_obj = self.kqd.get_pid_parameters()
            params_obj = self.kqd.get_output_parameters()
        pid_dict = {'p': pid_obj.p, 'i': pid_obj.i, 'd': pid_obj.d}
        param_names = ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out']
        params_dict = {name: getattr(params_obj, name) for name in param_names}
        return (pid_dict, params_dict)

    def set_fast_steering_params(self, pid, params):
        """Set fast steering PID and output parameters."""
        if not self.kqd:
            raise ConnectionError("Fast steering hardware not connected.")
        with self.kqd_lock:
            self.kqd.set_pid_parameters(p=pid['p'], i=pid['i'], d=pid['d'])
            self.kqd.set_output_parameters(
                xgain=params['xgain'], ygain=params['ygain'],
                xmin=params['xmin'], xmax=params['xmax'],
                ymin=params['ymin'], ymax=params['ymax']
            )

    def get_current_output(self):
        """Get current output voltages being sent to piezo actuators."""
        if not self.kqd:
            return None
        
        try:
            with self.kqd_lock:
                output = self.kqd.get_manual_output()
                
                # Named tuple is iterable, so we can index it
                if isinstance(output, (tuple, list)) and len(output) >= 2:
                    return {'xout': output[0], 'yout': output[1]}
                # Or use attributes directly
                elif hasattr(output, 'xpos') and hasattr(output, 'ypos'):
                    return {'xout': output.xpos, 'yout': output.ypos}
                else:
                    return None
                
        except Exception as e:
            print(f"Error getting output: {e}")
            return None
    # def debug_kqd_methods(self):
    #     """Temporary debug method to find output getter."""
    #     if not self.kqd:
    #         return
        
    #     with self.kqd_lock:
    #         print("\n=== KQD Available Methods ===")
    #         methods = [m for m in dir(self.kqd) if not m.startswith('_') and 'get' in m.lower()]
    #         for method in methods:
    #             print(f"  - {method}")
            
    #         print("\n=== Testing output parameters ===")
    #         params = self.kqd.get_output_parameters()
    #         print(f"Output params type: {type(params)}")
    #         print(f"Output params attributes: {dir(params)}")
    #         print(f"open_loop_out value: {params.open_loop_out}")
    #         print(f"open_loop_out type: {type(params.open_loop_out)}")

    def get_tic_settings(self):
        """Get Tic motor settings."""
        if not self.tic:
            raise ConnectionError("Tic not connected.")
        return {
            'velocity': self.tic.get_max_speed(),
            'current_limit': self.tic.get_current_limit(),
            'step_mode': self.tic.get_step_mode()
        }

    def set_tic_settings(self, velocity, current_limit, step_mode):
        """Set Tic motor settings."""
        if not self.tic:
            raise ConnectionError("Tic not connected.")
        self.tic.set_max_speed(velocity)
        self.tic.set_current_limit(current_limit)
        self.tic.set_step_mode(step_mode)

    def get_km_settings(self):
        """Get Kinesis Motor settings."""
        if not self.km:
            raise ConnectionError("Kinesis Motor not connected.")
        
        with self.km_lock:
            vel_params = self.km.get_velocity_parameters()
            return {
                'velocity': vel_params.max_velocity,
                'acceleration': vel_params.acceleration
            }

    def set_km_settings(self, velocity, acceleration):
        """Set Kinesis Motor settings."""
        if not self.km:
            raise ConnectionError("Kinesis Motor not connected.")
        
        with self.km_lock:
            self.km.setup_velocity(max_velocity=velocity, acceleration=acceleration)

    # ==================== MANUAL CONTROL ====================
    
    def manual_x(self, distance, relative=True):
        """Manually move X-axis (Tic)."""
        if not self.tic:
            raise ConnectionError("Tic not connected.")
        if relative:
            target_pos = self.tic.get_current_position() + distance
        else:
            target_pos = distance
        self.tic.set_target_position(target_pos)
        time.sleep(0.05)

    def manual_y(self, distance, relative=True):
        """Manually move Y-axis (Kinesis Motor)."""
        if not self.km:
            raise ConnectionError("Kinesis motor not connected.")
        
        with self.km_lock:
            if relative:
                target_pos = self.km.get_position() + distance
                self.km.move_to(target_pos)
            else:
                self.km.move_to(distance)
            self.km.wait_move()

    # ==================== CALIBRATION ====================
    
    def _manage_output_settings(self):
        """Helper for fast steering CLI settings."""
        try:
            with self.kqd_lock:
                current_params_obj = self.kqd.get_output_parameters()
            
            param_names = ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out']
            current = {name: getattr(current_params_obj, name) for name in param_names}
            print("\n--- Current Output Settings ---")
            for key, value in current.items():
                print(f"  - {key}: {value}")
            
            if input("\nUpdate settings? (y/n): ").lower() != 'y':
                return
            
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
    
    # def _manage_output_settings(self):
    #     """Helper for fast steering CLI settings."""
    #     try:
    #         with self.kqd_lock:
    #             current_params_obj = self.kqd.get_output_parameters()
            
    #         param_names = ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out']
    #         current = {name: getattr(current_params_obj, name) for name in param_names}
    #         print("\n--- Current Output Settings ---")
    #         for key, value in current.items():
    #             print(f"  - {key}: {value}")
            
    #         if input("\nUpdate settings? (y/n): ").lower() != 'y':
    #             return
            
    #         print("\nEnter new value or press Enter to keep current setting.")
    #         new_params = {}
    #         for key, value in current.items():
    #             if isinstance(value, (int, float)):
    #                 new_params[key] = float(input(f"  {key} [{value}]: ") or value)
    #             else:
    #                 new_params[key] = input(f"  {key} [{value}]: ") or value
            
    #         with self.kqd_lock:
    #             self.kqd.set_output_parameters(**new_params)
    #         print("\nSettings updated successfully.")
    #     except Exception as e:
    #         print(f"\nError managing settings: {e}", file=sys.stderr)
