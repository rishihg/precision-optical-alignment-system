import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import sys
from optical_aligner import OpticalAligner
from usbfinder import USBTTYFinder

# --- CONFIGURATION ---
usb = USBTTYFinder()
KQD_PORT = usb.find_by_product("Position Aligner")[0]
KM_PORT = usb.find_by_product("Brushed Motor Controller")[0]
# ---------------------

class TextRedirector:
    """A class to redirect stdout and stderr to a tkinter Text widget."""
    def __init__(self, widget):
        self.widget = widget

    def write(self, text):
        """Writes text to the widget in a thread-safe way."""
        # Use `after` to ensure the GUI is updated from the main thread
        self.widget.after(0, self._insert_text, text)

    def _insert_text(self, text):
        """The actual method that inserts text into the widget."""
        self.widget.configure(state='normal')
        self.widget.insert('end', text)
        self.widget.see('end')  # Auto-scroll to the bottom
        self.widget.configure(state='disabled')

    def flush(self):
        """Required for the stream object interface."""
        pass

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Optical Alignment Control")
        self.geometry("620x850") # Increased height for the log window

        self.aligner = OpticalAligner(kqd_port=KQD_PORT, km_port=KM_PORT)
        self.slow_steering_connected = False
        self.fast_steering_connected = False
        self.loop_thread = None

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(pady=10, padx=10, fill="both", expand=True)

        self._create_slow_steering_tab()
        self._create_fast_steering_tab()
        self._create_manual_control_tab()
        self._create_log_output_window() # NEW: Create the log window

        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _create_log_output_window(self):
        """Creates the text widget for logging and redirects stdout/stderr."""
        log_frame = ttk.LabelFrame(self, text="Log Output")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        log_widget = tk.Text(log_frame, height=12, wrap='word', state='disabled', bg="#f0f0f0")
        
        scrollbar = ttk.Scrollbar(log_frame, command=log_widget.yview)
        scrollbar.pack(side='right', fill='y')
        log_widget.pack(side='left', fill="both", expand=True, padx=5, pady=5)
        
        log_widget['yscrollcommand'] = scrollbar.set

        # Redirect stdout and stderr to the text widget
        sys.stdout = TextRedirector(log_widget)
        sys.stderr = TextRedirector(log_widget)

        print("--- GUI Initialized ---")
        print("Log output will now appear in this window.")


    def _create_slow_steering_tab(self):
        slow_tab = ttk.Frame(self.notebook)
        self.notebook.add(slow_tab, text="Slow Steering")

        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(slow_tab, text="Connection")
        conn_frame.pack(fill="x", padx=5, pady=5)
        self.slow_conn_btn = ttk.Button(conn_frame, text="Connect", command=self._connect_slow)
        self.slow_conn_btn.pack(side="left", padx=5, pady=5)
        self.slow_disconn_btn = ttk.Button(conn_frame, text="Disconnect", command=self._disconnect_slow, state="disabled")
        self.slow_disconn_btn.pack(side="left", padx=5, pady=5)

        # --- Calibration Frame ---
        cal_frame = ttk.LabelFrame(slow_tab, text="Calibration")
        cal_frame.pack(fill="x", padx=5, pady=5)
        self.cal_btn = ttk.Button(cal_frame, text="Run Calibration", command=self._run_calibration, state="disabled")
        self.cal_btn.pack(side="left", padx=5, pady=5)
        self.recal_sum_btn = ttk.Button(cal_frame, text="Recalibrate Baseline", command=self._recalibrate_sum, state="disabled")
        self.recal_sum_btn.pack(side="left", padx=5, pady=5)


        # --- Settings Frame ---
        settings_frame = ttk.LabelFrame(slow_tab, text="Settings")
        settings_frame.pack(fill="x", padx=5, pady=5, expand=True)
        
        # PID
        ttk.Label(settings_frame, text="X PID (P/I/D):").grid(row=0, column=0, sticky="w", padx=5)
        self.pid_x_p = tk.StringVar(value="0.1")
        self.pid_x_i = tk.StringVar(value="0.1")
        self.pid_x_d = tk.StringVar(value="0.05")
        ttk.Entry(settings_frame, textvariable=self.pid_x_p, width=5).grid(row=0, column=1)
        ttk.Entry(settings_frame, textvariable=self.pid_x_i, width=5).grid(row=0, column=2)
        ttk.Entry(settings_frame, textvariable=self.pid_x_d, width=5).grid(row=0, column=3)

        ttk.Label(settings_frame, text="Y PID (P/I/D):").grid(row=1, column=0, sticky="w", padx=5)
        self.pid_y_p = tk.StringVar(value="0.1")
        self.pid_y_i = tk.StringVar(value="0.1")
        self.pid_y_d = tk.StringVar(value="0.05")
        ttk.Entry(settings_frame, textvariable=self.pid_y_p, width=5).grid(row=1, column=1)
        ttk.Entry(settings_frame, textvariable=self.pid_y_i, width=5).grid(row=1, column=2)
        ttk.Entry(settings_frame, textvariable=self.pid_y_d, width=5).grid(row=1, column=3)

        # Velocities
        ttk.Label(settings_frame, text="X-Axis Velocity:").grid(row=2, column=0, sticky="w", padx=5)
        self.tic_vel = tk.StringVar(value="2000000")
        ttk.Entry(settings_frame, textvariable=self.tic_vel, width=10).grid(row=2, column=1, columnspan=2, sticky="w")
        
        ttk.Label(settings_frame, text="Y-Axis Velocity:").grid(row=3, column=0, sticky="w", padx=5)
        self.km_vel = tk.StringVar(value="2.0")
        ttk.Entry(settings_frame, textvariable=self.km_vel, width=10).grid(row=3, column=1, columnspan=2, sticky="w")

        # Tic Specific
        ttk.Label(settings_frame, text="X-Axis Current (mA):").grid(row=4, column=0, sticky="w", padx=5)
        self.tic_current = tk.StringVar(value="1000")
        ttk.Entry(settings_frame, textvariable=self.tic_current, width=10).grid(row=4, column=1, columnspan=2, sticky="w")
        
        ttk.Label(settings_frame, text="X-Axis Step Mode:").grid(row=5, column=0, sticky="w", padx=5)
        self.tic_step = tk.StringVar(value="5")
        ttk.Combobox(settings_frame, textvariable=self.tic_step, values=[str(i) for i in range(6)], width=8).grid(row=5, column=1, columnspan=2, sticky="w")
        
        self.get_settings_btn = ttk.Button(settings_frame, text="Get Current", command=self._get_slow_settings, state="disabled")
        self.get_settings_btn.grid(row=6, column=0, pady=10)
        self.apply_settings_btn = ttk.Button(settings_frame, text="Apply Settings", command=self._apply_slow_settings, state="disabled")
        self.apply_settings_btn.grid(row=6, column=1, pady=10)

        # --- Feedback Loop Frame ---
        loop_frame = ttk.LabelFrame(slow_tab, text="Feedback Loop Control")
        loop_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(loop_frame, text="Master Loop Gain:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.loop_gain_var = tk.StringVar(value="1.0")
        ttk.Entry(loop_frame, textvariable=self.loop_gain_var, width=8).grid(row=0, column=1, padx=5, pady=5)
        
        self.start_loop_btn = ttk.Button(loop_frame, text="Start Slow Loop", command=self._start_slow_loop, state="disabled")
        self.start_loop_btn.grid(row=1, column=0, padx=5, pady=5)
        self.stop_loop_btn = ttk.Button(loop_frame, text="Stop Loop", command=self._stop_slow_loop, state="disabled")
        self.stop_loop_btn.grid(row=1, column=1, padx=5, pady=5)
        
        self.loop_status_var = tk.StringVar(value="Status: Idle")
        ttk.Label(loop_frame, textvariable=self.loop_status_var).grid(row=2, column=0, columnspan=2, sticky="w", padx=5)


    def _create_fast_steering_tab(self):
        fast_tab = ttk.Frame(self.notebook)
        self.notebook.add(fast_tab, text="Fast Steering")
        ttk.Label(fast_tab, text="Fast steering controls will be implemented here.").pack(pady=20)

    def _create_manual_control_tab(self):
        manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(manual_tab, text="Manual Control")

        # --- X-Axis Frame ---
        x_frame = ttk.LabelFrame(manual_tab, text="X-Axis Control")
        x_frame.pack(fill="x", padx=5, pady=5, expand=True)
        ttk.Label(x_frame, text="Magnitude (steps):").grid(row=0, column=0, padx=5, pady=5)
        self.x_move_var = tk.StringVar(value="100")
        ttk.Entry(x_frame, textvariable=self.x_move_var, width=10).grid(row=0, column=1)
        
        self.x_plus_btn = ttk.Button(x_frame, text="+X", command=lambda: self._manual_move('x', 1), state="disabled")
        self.x_plus_btn.grid(row=0, column=2, padx=5)
        self.x_minus_btn = ttk.Button(x_frame, text="-X", command=lambda: self._manual_move('x', -1), state="disabled")
        self.x_minus_btn.grid(row=0, column=3, padx=5)

        # --- Y-Axis Frame ---
        y_frame = ttk.LabelFrame(manual_tab, text="Y-Axis Control")
        y_frame.pack(fill="x", padx=5, pady=5, expand=True)
        ttk.Label(y_frame, text="Magnitude (steps):").grid(row=0, column=0, padx=5, pady=5)
        self.y_move_var = tk.StringVar(value="1000")
        ttk.Entry(y_frame, textvariable=self.y_move_var, width=10).grid(row=0, column=1)

        self.y_plus_btn = ttk.Button(y_frame, text="+Y", command=lambda: self._manual_move('y', 1), state="disabled")
        self.y_plus_btn.grid(row=0, column=2, padx=5)
        self.y_minus_btn = ttk.Button(y_frame, text="-Y", command=lambda: self._manual_move('y', -1), state="disabled")
        self.y_minus_btn.grid(row=0, column=3, padx=5)
        
        # --- Soft Limits ---
        x_limit_frame = ttk.LabelFrame(manual_tab, text="X-Axis Soft Limits")
        x_limit_frame.pack(fill="x", padx=5, pady=5, expand=True)
        ttk.Label(x_limit_frame, text="Range (+/- steps):").grid(row=0, column=0, padx=5, pady=5)
        self.x_limit_range = tk.StringVar(value="500")
        ttk.Entry(x_limit_frame, textvariable=self.x_limit_range, width=10).grid(row=0, column=1)
        self.set_x_limit_btn = ttk.Button(x_limit_frame, text="Set Limits", command=lambda: self._set_soft_limits('x'), state="disabled")
        self.set_x_limit_btn.grid(row=0, column=2, padx=5)
        self.clear_x_limit_btn = ttk.Button(x_limit_frame, text="Clear Limits", command=lambda: self._clear_soft_limits('x'), state="disabled")
        self.clear_x_limit_btn.grid(row=0, column=3, padx=5)
        
        y_limit_frame = ttk.LabelFrame(manual_tab, text="Y-Axis Soft Limits")
        y_limit_frame.pack(fill="x", padx=5, pady=5, expand=True)
        ttk.Label(y_limit_frame, text="Range (+/- steps):").grid(row=0, column=0, padx=5, pady=5)
        self.y_limit_range = tk.StringVar(value="2000")
        ttk.Entry(y_limit_frame, textvariable=self.y_limit_range, width=10).grid(row=0, column=1)
        self.set_y_limit_btn = ttk.Button(y_limit_frame, text="Set Limits", command=lambda: self._set_soft_limits('y'), state="disabled")
        self.set_y_limit_btn.grid(row=0, column=2, padx=5)
        self.clear_y_limit_btn = ttk.Button(y_limit_frame, text="Clear Limits", command=lambda: self._clear_soft_limits('y'), state="disabled")
        self.clear_y_limit_btn.grid(row=0, column=3, padx=5)


    def _connect_slow(self):
        try:
            self.aligner.connect_slow_steering()
            self.slow_steering_connected = True
            # Update button states
            self.slow_conn_btn.config(state="disabled")
            self.slow_disconn_btn.config(state="normal")
            # Enable all controls
            for btn in [self.cal_btn, self.recal_sum_btn, self.start_loop_btn, self.x_plus_btn, self.x_minus_btn, self.y_plus_btn, self.y_minus_btn, self.get_settings_btn, self.apply_settings_btn, self.set_x_limit_btn, self.clear_x_limit_btn, self.set_y_limit_btn, self.clear_y_limit_btn]:
                btn.config(state="normal")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")

    def _disconnect_slow(self):
        self._stop_slow_loop()
        self.aligner.disconnect_slow_steering()
        self.slow_steering_connected = False
        # Update button states
        self.slow_conn_btn.config(state="normal")
        self.slow_disconn_btn.config(state="disabled")
        # Disable all controls
        for btn in [self.cal_btn, self.recal_sum_btn, self.start_loop_btn, self.stop_loop_btn, self.x_plus_btn, self.x_minus_btn, self.y_plus_btn, self.y_minus_btn, self.get_settings_btn, self.apply_settings_btn, self.set_x_limit_btn, self.clear_x_limit_btn, self.set_y_limit_btn, self.clear_y_limit_btn]:
            btn.config(state="disabled")
        
    def _run_in_thread(self, target_func, on_complete):
        def task_wrapper():
            try:
                target_func()
                self.after(0, on_complete, None)
            except Exception as e:
                self.after(0, on_complete, e)
        threading.Thread(target=task_wrapper, daemon=True).start()

    def _run_calibration(self):
        self.cal_btn.config(state="disabled")
        print("\n--- Starting system calibration ---")
        
        def on_complete(error):
            self.cal_btn.config(state="normal")
            if error:
                print(f"ERROR: Calibration failed: {error}")
                messagebox.showerror("Calibration Error", f"Calibration failed: {error}")
            else:
                print("--- Calibration complete ---")

        self._run_in_thread(self.aligner.calibrate, on_complete)

    def _recalibrate_sum(self):
        try:
            self.aligner.recalibrate_sum_baseline()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to recalibrate baseline: {e}")

    def _start_slow_loop(self):
        if self.loop_thread and self.loop_thread.is_alive():
            return
        try:
            gain = float(self.loop_gain_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Master Loop Gain must be a valid number.")
            return
            
        self.aligner.stop_loop_event.clear()
        self.loop_thread = threading.Thread(target=self._run_loop_thread, args=(gain,), daemon=True)
        self.loop_thread.start()
        
        self.start_loop_btn.config(state="disabled")
        self.stop_loop_btn.config(state="normal")
        self.loop_status_var.set("Status: Running...")

    def _run_loop_thread(self, gain):
        status = self.aligner.run_slow_feedback_loop(loop_gain=gain)
        status_map = {
            "stopped_beam_lost": "Stopped: Beam Lost",
            "stopped_by_user": "Stopped: User Interrupt",
            "error_not_calibrated": "Error: Not Calibrated",
            "error_no_baseline": "Error: No Baseline"
        }
        self.after(0, self._update_loop_status_on_stop, status_map.get(status, "Idle"))

    def _update_loop_status_on_stop(self, status):
        self.loop_status_var.set(f"Status: {status}")
        self.start_loop_btn.config(state="normal")
        self.stop_loop_btn.config(state="disabled")

    def _stop_slow_loop(self):
        if self.loop_thread and self.loop_thread.is_alive():
            self.aligner.stop_loop_event.set()

    def _manual_move(self, axis, direction):
        try:
            if axis == 'x':
                distance = int(self.x_move_var.get())
                self.aligner.manual_x(distance * direction)
            elif axis == 'y':
                distance = int(self.y_move_var.get())
                self.aligner.manual_y(distance * direction)
        except Exception as e:
            messagebox.showerror("Move Error", f"Failed to move: {e}")

    def _get_slow_settings(self):
        try:
            pid = self.aligner.get_pid_settings()
            tic = self.aligner.get_tic_settings()
            km = self.aligner.get_km_settings()
            
            self.pid_x_p.set(str(pid['x']['p']))
            self.pid_x_i.set(str(pid['x']['i']))
            self.pid_x_d.set(str(pid['x']['d']))
            self.pid_y_p.set(str(pid['y']['p']))
            self.pid_y_i.set(str(pid['y']['i']))
            self.pid_y_d.set(str(pid['y']['d']))
            self.tic_vel.set(str(tic['velocity']))
            self.km_vel.set(str(km['velocity']))
            self.tic_current.set(str(tic['current_limit']))
            self.tic_step.set(str(tic['step_mode']))
        except Exception as e:
            messagebox.showerror("Error", f"Could not get settings: {e}")

    def _apply_slow_settings(self):
        try:
            self.aligner.set_pid_settings(
                float(self.pid_x_p.get()), float(self.pid_x_i.get()), float(self.pid_x_d.get()),
                float(self.pid_y_p.get()), float(self.pid_y_i.get()), float(self.pid_y_d.get())
            )
            self.aligner.set_tic_settings(
                int(self.tic_vel.get()), int(self.tic_current.get()), int(self.tic_step.get())
            )
            self.aligner.set_km_settings(float(self.km_vel.get()))
            messagebox.showinfo("Success", "Settings applied successfully.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not apply settings: {e}")

    def _set_soft_limits(self, axis):
        try:
            if axis == 'x':
                range_val = int(self.x_limit_range.get())
                current_pos = self.aligner.tic.get_current_position()
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                self.aligner.set_tic_soft_limits(min_pos, max_pos)
            elif axis == 'y':
                range_val = int(self.y_limit_range.get())
                current_pos = self.aligner.km.get_position()
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                self.aligner.set_km_soft_limits(min_pos, max_pos)
        except Exception as e:
            messagebox.showerror("Error", f"Could not set limits: {e}")

    def _clear_soft_limits(self, axis):
        try:
            if axis == 'x':
                self.aligner.clear_tic_soft_limits()
            elif axis == 'y':
                self.aligner.clear_km_soft_limits()
        except Exception as e:
            messagebox.showerror("Error", f"Could not clear limits: {e}")

    def _on_closing(self):
        if self.slow_steering_connected:
            self._disconnect_slow()
        if self.fast_steering_connected:
            self.aligner.disconnect_fast_steering()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()

