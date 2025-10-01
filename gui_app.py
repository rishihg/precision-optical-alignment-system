import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
from optical_aligner import OpticalAligner
from collections import deque
import sys
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# --- CONFIGURATION ---
# This is now controlled by a checkbox in the GUI
# ---------------------

# --- Hardware Discovery & Startup ---
startup_error = None
KQD_PORT, KM_PORT = None, None
SIMULATION = '--simulation' in sys.argv

if SIMULATION:
    print("INFO: Starting in simulation mode.")
    KQD_PORT, KM_PORT = "SIM_KQD", "SIM_KM"
else:
    try:
        from usbfinder import USBTTYFinder
        usb = USBTTYFinder()
        try:
            KQD_PORT = usb.find_by_product("Position Aligner")[0]
            print(f"INFO: Found KQD at {KQD_PORT}")
        except Exception as e:
            raise ConnectionError(f"Could not find KQD (Position Aligner). This is essential hardware.\nError: {e}")

        try:
            KM_PORT = usb.find_by_product("Brushed Motor Controller")[0]
            print(f"INFO: Found Kinesis Motor at {KM_PORT}")
        except Exception as e:
            print(f"WARNING: Kinesis Motor not found. Slow/Manual steering will be disabled.\nError: {e}")
            KM_PORT = None

    except Exception as e:
        startup_error = e 
# ---------------------


class PlottingWindow(tk.Toplevel):
    """A separate, dedicated window for real-time plotting."""
    def __init__(self, master):
        super().__init__(master)
        self.title("Live Error Plot")
        self.geometry("600x600")

        self.is_plotting_active = True
        self.plot_time_data, self.x_error_data, self.y_error_data = deque(), deque(), deque()

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 6))
        self.fig.tight_layout(pad=3.0)
        self.ax1.set_title("X-Axis Error"); self.ax1.set_ylabel("Error (V)")
        self.line1, = self.ax1.plot([], [], 'r-')
        self.ax1.axhline(0, color='gray', linestyle='--', linewidth=1)

        self.ax2.set_title("Y-Axis Error"); self.ax2.set_xlabel("Time (s)"); self.ax2.set_ylabel("Error (V)")
        self.line2, = self.ax2.plot([], [], 'b-')
        self.ax2.axhline(0, color='gray', linestyle='--', linewidth=1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        toolbar = NavigationToolbar2Tk(self.canvas, self)
        toolbar.update()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        control_frame = ttk.Frame(self)
        control_frame.pack(side=tk.BOTTOM, pady=5)
        self.toggle_btn = ttk.Button(control_frame, text="Stop Live Tracking", command=self._toggle_live_plotting)
        self.toggle_btn.pack(side="left", padx=5)
        clear_btn = ttk.Button(control_frame, text="Clear Plot Data", command=self._clear_plot_data)
        clear_btn.pack(side="left", padx=5)
        
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _toggle_live_plotting(self):
        self.is_plotting_active = not self.is_plotting_active
        self.toggle_btn.config(text="Stop Live Tracking" if self.is_plotting_active else "Start Live Tracking")

    def _clear_plot_data(self):
        self.plot_time_data.clear(); self.x_error_data.clear(); self.y_error_data.clear()
        self.line1.set_data([], []); self.line2.set_data([], [])
        for ax in [self.ax1, self.ax2]: ax.relim(); ax.autoscale_view()
        self.canvas.draw()

    def update_plot(self, elapsed_time, x_err, y_err):
        self.plot_time_data.append(elapsed_time)
        self.x_error_data.append(x_err)
        self.y_error_data.append(y_err)

        if self.is_plotting_active:
            self.line1.set_data(self.plot_time_data, self.x_error_data)
            self.line2.set_data(self.plot_time_data, self.y_error_data)
            for ax in [self.ax1, self.ax2]: ax.relim(); ax.autoscale_view()
            self.canvas.draw()
            
    def _on_closing(self):
        self.master.plotting_window = None
        self.destroy()

class SlowSteeringWindow(tk.Toplevel):
    def __init__(self, master, aligner):
        super().__init__(master)
        self.title("Slow Steering Control")
        self.aligner = aligner
        self.master_app = master

        cal_frame = ttk.LabelFrame(self, text="Calibration")
        cal_frame.pack(fill="x", padx=5, pady=5)
        self.cal_x_btn = ttk.Button(cal_frame, text="Calibrate X-Axis", command=lambda: self.master_app._run_axis_calibration('x', self.cal_x_btn))
        self.cal_x_btn.pack(side="left", padx=5, pady=5)
        self.cal_y_btn = ttk.Button(cal_frame, text="Calibrate Y-Axis", command=lambda: self.master_app._run_axis_calibration('y', self.cal_y_btn))
        self.cal_y_btn.pack(side="left", padx=5, pady=5)
        self.recal_sum_btn = ttk.Button(cal_frame, text="Recalibrate Signal Baseline", command=self.master_app._recalibrate_sum)
        self.recal_sum_btn.pack(side="left", padx=5, pady=5)
        
        data_frame = ttk.LabelFrame(self, text="Calibration Data")
        data_frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(data_frame, text="Save...", command=self.master_app._save_calibration).pack(side="left", padx=5)
        ttk.Button(data_frame, text="Load...", command=self.master_app._load_calibration).pack(side="left", padx=5)
        ttk.Button(data_frame, text="Clear", command=self.master_app._clear_calibration).pack(side="left", padx=5)

        settings_frame = ttk.LabelFrame(self, text="Settings")
        settings_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(settings_frame, text="X PID (P/I/D):").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(settings_frame, textvariable=self.master_app.pid_x_p, width=5).grid(row=0, column=1); ttk.Entry(settings_frame, textvariable=self.master_app.pid_x_i, width=5).grid(row=0, column=2); ttk.Entry(settings_frame, textvariable=self.master_app.pid_x_d, width=5).grid(row=0, column=3)
        ttk.Label(settings_frame, text="Y PID (P/I/D):").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(settings_frame, textvariable=self.master_app.pid_y_p, width=5).grid(row=1, column=1); ttk.Entry(settings_frame, textvariable=self.master_app.pid_y_i, width=5).grid(row=1, column=2); ttk.Entry(settings_frame, textvariable=self.master_app.pid_y_d, width=5).grid(row=1, column=3)
        ttk.Label(settings_frame, text="X-Axis Velocity:").grid(row=2, column=0, sticky="w", padx=5)
        ttk.Entry(settings_frame, textvariable=self.master_app.tic_vel, width=10).grid(row=2, column=1, columnspan=2, sticky="w")
        ttk.Label(settings_frame, text="Y-Axis Velocity:").grid(row=3, column=0, sticky="w", padx=5)
        ttk.Entry(settings_frame, textvariable=self.master_app.km_vel, width=10).grid(row=3, column=1, columnspan=2, sticky="w")
        ttk.Label(settings_frame, text="X-Axis Current (mA):").grid(row=4, column=0, sticky="w", padx=5)
        ttk.Entry(settings_frame, textvariable=self.master_app.tic_current, width=10).grid(row=4, column=1, columnspan=2, sticky="w")
        ttk.Label(settings_frame, text="X-Axis Step Mode:").grid(row=5, column=0, sticky="w", padx=5)
        ttk.Combobox(settings_frame, textvariable=self.master_app.tic_step_str, values=list(self.master_app.step_mode_map.keys()), width=12, state="readonly").grid(row=5, column=1, columnspan=2, sticky="w")
        ttk.Button(settings_frame, text="Get Current", command=self.master_app._get_slow_settings).grid(row=6, column=0, pady=5, sticky="w")
        ttk.Button(settings_frame, text="Apply Settings", command=self.master_app._apply_slow_settings).grid(row=6, column=1, pady=5, sticky="w")
        
        loop_frame = ttk.LabelFrame(self, text="Feedback Loop Control")
        loop_frame.pack(fill="x", padx=5, pady=5)
        
        gain_frame = ttk.Frame(loop_frame)
        gain_frame.pack(pady=2, fill="x")
        ttk.Label(gain_frame, text="X-Axis Loop Gain:").pack(side="left", padx=5)
        ttk.Entry(gain_frame, textvariable=self.master_app.loop_gain_x_var, width=8).pack(side="left")
        ttk.Label(gain_frame, text="Y-Axis Loop Gain:").pack(side="left", padx=(10, 5))
        ttk.Entry(gain_frame, textvariable=self.master_app.loop_gain_y_var, width=8).pack(side="left")

        button_frame = ttk.Frame(loop_frame)
        button_frame.pack(pady=2, fill="x")
        self.start_loop_btn = ttk.Button(button_frame, text="Start Slow Loop", command=self.master_app._start_slow_loop)
        self.start_loop_btn.pack(side="left", padx=5)
        self.stop_loop_btn = ttk.Button(button_frame, text="Stop Loop", command=self.master_app._stop_slow_loop, state="disabled")
        self.stop_loop_btn.pack(side="left", padx=5)

        ttk.Label(loop_frame, textvariable=self.master_app.loop_status_var).pack(pady=2)

        self.protocol("WM_DELETE_WINDOW", self._on_closing)
        
    def _on_closing(self):
        self.master_app.slow_steering_window = None
        self.destroy()

class FastSteeringWindow(tk.Toplevel):
    def __init__(self, master, aligner):
        super().__init__(master)
        self.title("Fast Steering Control")
        self.aligner = aligner
        self.master_app = master

        readout_frame = ttk.LabelFrame(self, text="Live Readout")
        readout_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(readout_frame, textvariable=self.master_app.fast_status_var).pack(anchor="w", padx=5)
        
        mode_frame = ttk.LabelFrame(self, text="Mode Control")
        mode_frame.pack(fill="x", padx=5, pady=5)
        
        ttk.Radiobutton(mode_frame, text="Monitor", variable=self.master_app.fast_mode_var, value="monitor", command=self.master_app._apply_fast_mode).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="Open Loop", variable=self.master_app.fast_mode_var, value="open_loop", command=self.master_app._apply_fast_mode).pack(side="left", padx=5)
        ttk.Radiobutton(mode_frame, text="Closed Loop", variable=self.master_app.fast_mode_var, value="closed_loop", command=self.master_app._apply_fast_mode).pack(side="left", padx=5)

        params_frame = ttk.LabelFrame(self, text="Parameters")
        params_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(params_frame, text="CL PID (P/I/D):").grid(row=0, column=0, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_p, width=5).grid(row=0, column=1); ttk.Entry(params_frame, textvariable=self.master_app.fast_i, width=5).grid(row=0, column=2); ttk.Entry(params_frame, textvariable=self.master_app.fast_d, width=5).grid(row=0, column=3)
        ttk.Label(params_frame, text="OL Voltage (X/Y):").grid(row=1, column=0, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_vx, width=5).grid(row=1, column=1); ttk.Entry(params_frame, textvariable=self.master_app.fast_vy, width=5).grid(row=1, column=2)
        ttk.Label(params_frame, text="X Gain (-1 to 1):").grid(row=2, column=0, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_xgain, width=5).grid(row=2, column=1)
        ttk.Label(params_frame, text="Y Gain (-1 to 1):").grid(row=2, column=2, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_ygain, width=5).grid(row=2, column=3)
        ttk.Label(params_frame, text="X Range (V):").grid(row=3, column=0, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_xmin, width=5).grid(row=3, column=1); ttk.Entry(params_frame, textvariable=self.master_app.fast_xmax, width=5).grid(row=3, column=2)
        ttk.Label(params_frame, text="Y Range (V):").grid(row=4, column=0, sticky="w", padx=5)
        ttk.Entry(params_frame, textvariable=self.master_app.fast_ymin, width=5).grid(row=4, column=1); ttk.Entry(params_frame, textvariable=self.master_app.fast_ymax, width=5).grid(row=4, column=2)
        ttk.Button(params_frame, text="Get Current", command=self.master_app._get_fast_settings).grid(row=5, column=0, pady=10)
        ttk.Button(params_frame, text="Apply Settings", command=self.master_app._apply_fast_settings).grid(row=5, column=1, pady=10)

        self.protocol("WM_DELETE_WINDOW", self._on_closing)
    def _on_closing(self):
        self.master_app.fast_steering_window = None
        self.destroy()

class ManualControlWindow(tk.Toplevel):
    def __init__(self, master, aligner):
        super().__init__(master)
        self.title("Manual Control")
        self.aligner = aligner
        self.master_app = master

        x_frame = ttk.LabelFrame(self, text="X-Axis Control")
        x_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(x_frame, text="Magnitude (steps):").grid(row=0, column=0, padx=5, pady=5)
        self.x_move_var = tk.StringVar(value="100")
        ttk.Entry(x_frame, textvariable=self.x_move_var, width=10).grid(row=0, column=1)
        ttk.Button(x_frame, text="+X", command=lambda: self.master_app._manual_move('x', 1, self.x_move_var)).grid(row=0, column=2, padx=5)
        ttk.Button(x_frame, text="-X", command=lambda: self.master_app._manual_move('x', -1, self.x_move_var)).grid(row=0, column=3, padx=5)
        y_frame = ttk.LabelFrame(self, text="Y-Axis Control")
        y_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(y_frame, text="Magnitude (steps):").grid(row=0, column=0, padx=5, pady=5)
        self.y_move_var = tk.StringVar(value="1000")
        ttk.Entry(y_frame, textvariable=self.y_move_var, width=10).grid(row=0, column=1)
        ttk.Button(y_frame, text="+Y", command=lambda: self.master_app._manual_move('y', 1, self.y_move_var)).grid(row=0, column=2, padx=5)
        ttk.Button(y_frame, text="-Y", command=lambda: self.master_app._manual_move('y', -1, self.y_move_var)).grid(row=0, column=3, padx=5)

        self.protocol("WM_DELETE_WINDOW", self._on_closing)
    def _on_closing(self):
        self.master_app.manual_control_window = None
        self.destroy()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Optical Alignment Control - Launcher")
        self.geometry("1000x600")

        self.aligner = None 
        self.is_connected = False
        self.loop_thread = None
        self.fast_polling_active = False
        
        self.plotting_window = None
        self.slow_steering_window = None
        self.fast_steering_window = None
        self.manual_control_window = None

        self.last_fast_update_time = 0
        self.gui_update_interval = 0.25
        self.kqd_lock = threading.Lock()

        # --- Setup ALL StringVars in the main App ---
        self.pid_x_p, self.pid_x_i, self.pid_x_d = tk.StringVar(value="0.1"), tk.StringVar(value="0.1"), tk.StringVar(value="0.05")
        self.pid_y_p, self.pid_y_i, self.pid_y_d = tk.StringVar(value="0.1"), tk.StringVar(value="0.1"), tk.StringVar(value="0.05")
        self.tic_vel = tk.StringVar(value="2000000"); self.km_vel = tk.StringVar(value="2.0")
        self.tic_current = tk.StringVar(value="1000")
        self.step_mode_map = {"Full step": 0, "1/2 step": 1, "1/4 step": 2, "1/8 step": 3, "1/16 step": 4, "1/32 step": 5}
        self.inv_step_mode_map = {v: k for k, v in self.step_mode_map.items()}
        self.tic_step_str = tk.StringVar(value="1/32 step")
        self.loop_gain_x_var = tk.StringVar(value="1.0"); self.loop_gain_y_var = tk.StringVar(value="1.0")
        self.loop_status_var = tk.StringVar(value="Status: Idle")
        self.fast_status_var = tk.StringVar(value="Status: Disconnected | X: --- | Y: --- | Sum: ---")
        self.fast_mode_var = tk.StringVar(value="open_loop")
        self.fast_p, self.fast_i, self.fast_d = tk.StringVar(value="1.0"), tk.StringVar(value="0.0"), tk.StringVar(value="0.0")
        self.fast_vx, self.fast_vy = tk.StringVar(value="0.0"), tk.StringVar(value="0.0")
        self.fast_xgain, self.fast_ygain = tk.StringVar(value="1.0"), tk.StringVar(value="1.0")
        self.fast_xmin, self.fast_xmax = tk.StringVar(value="-10.0"), tk.StringVar(value="10.0")
        self.fast_ymin, self.fast_ymax = tk.StringVar(value="-10.0"), tk.StringVar(value="10.0")
        # ---------------------------------------------

        self.paned_window = ttk.PanedWindow(self, orient=tk.VERTICAL)
        self.paned_window.pack(fill="both", expand=True)
        main_controls_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(main_controls_frame, weight=3)
        self._create_log_output_panel(self.paned_window)
        self._create_main_launcher(main_controls_frame)
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _create_log_output_panel(self, parent):
        log_frame = ttk.LabelFrame(parent, text="Log Output")
        parent.add(log_frame, weight=1)
        self.log_text = tk.Text(log_frame, height=8, state="disabled")
        self.log_text.pack(padx=5, pady=5, fill="both", expand=True)
        sys.stdout = self.TextRedirector(self.log_text, "stdout")
        sys.stderr = self.TextRedirector(self.log_text, "stderr")
    
    class TextRedirector(object):
        def __init__(self, widget, tag="stdout"):
            self.widget = widget; self.tag = tag
        def write(self, str_):
            self.widget.config(state="normal")
            self.widget.insert("end", str_, (self.tag,))
            self.widget.see("end")
            self.widget.config(state="disabled")
        def flush(self): pass
    
    def _create_main_launcher(self, parent):
        launcher_frame = ttk.LabelFrame(parent, text="Main Controls")
        launcher_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        conn_frame = ttk.LabelFrame(launcher_frame, text="Hardware Connections")
        conn_frame.pack(pady=5, fill="x")
        self.simulation_var = tk.BooleanVar(value=SIMULATION)
        ttk.Checkbutton(conn_frame, text="Run in Simulation Mode", variable=self.simulation_var, command=self._on_sim_toggle).pack(side="top", anchor="w")
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect All Hardware", command=self._connect_all)
        self.connect_btn.pack(side="left", padx=5, pady=5)
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect All Hardware", command=self._disconnect_all, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5, pady=5)

        window_frame = ttk.LabelFrame(launcher_frame, text="Control Panels")
        window_frame.pack(pady=5, fill="x")
        self.open_slow_btn = ttk.Button(window_frame, text="Open Slow Control Panel", command=self._open_slow_steering_window, state="disabled")
        self.open_slow_btn.pack(pady=5)
        self.open_fast_btn = ttk.Button(window_frame, text="Open Fast Control Panel", command=self._open_fast_steering_window, state="disabled")
        self.open_fast_btn.pack(pady=5)
        self.open_manual_btn = ttk.Button(window_frame, text="Open Manual Control Panel", command=self._open_manual_control_window, state="disabled")
        self.open_manual_btn.pack(pady=5)
        self.open_plot_btn = ttk.Button(window_frame, text="Open Live Plot", command=self._open_plot_window, state="disabled")
        self.open_plot_btn.pack(pady=5)

    def _on_sim_toggle(self):
        if self.is_connected:
            messagebox.showwarning("Warning", "Cannot change simulation mode while hardware is connected.")
            self.simulation_var.set(not self.simulation_var.get())
            
    def _open_slow_steering_window(self):
        if self.slow_steering_window is None or not self.slow_steering_window.winfo_exists(): self.slow_steering_window = SlowSteeringWindow(self, self.aligner)
        else: self.slow_steering_window.lift()
    def _open_fast_steering_window(self):
        if self.fast_steering_window is None or not self.fast_steering_window.winfo_exists(): self.fast_steering_window = FastSteeringWindow(self, self.aligner)
        else: self.fast_steering_window.lift()
    def _open_manual_control_window(self):
        if self.manual_control_window is None or not self.manual_control_window.winfo_exists(): self.manual_control_window = ManualControlWindow(self, self.aligner)
        else: self.manual_control_window.lift()
    def _open_plot_window(self):
        if self.plotting_window is None or not self.plotting_window.winfo_exists():
            self.plotting_window = PlottingWindow(self)
        else:
            self.plotting_window.lift()

    def _connect_all(self):
        if self.is_connected: self._disconnect_all()
        try:
            is_sim = self.simulation_var.get()
            kqd_port, km_port = self._find_hardware(is_sim)
            self.aligner = OpticalAligner(kqd_port=kqd_port, km_port=km_port, simulation=is_sim)
            self.aligner.connect_all()
            self.is_connected = True
            if self.aligner.kqd:
                self.open_fast_btn.config(state="normal")
                self.open_plot_btn.config(state="normal")
                self.fast_polling_active = True
                threading.Thread(target=self._poll_fast_readings, daemon=True).start()
            if self.aligner.km and self.aligner.tic:
                self.open_slow_btn.config(state="normal")
                self.open_manual_btn.config(state="normal")
            self.disconnect_btn.config(state="normal"); self.connect_btn.config(state="disabled")
            self.simulation_var.set(is_sim)
            messagebox.showinfo("Success", "Hardware connected successfully.")
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.is_connected = False

    def _disconnect_all(self):
        self._stop_slow_loop(); self._stop_fast_polling()
        for win in [self.slow_steering_window, self.fast_steering_window, self.manual_control_window, self.plotting_window]:
            if win and win.winfo_exists(): win.destroy()
        if self.aligner: self.aligner.disconnect_all()
        self.is_connected = False
        for btn in [self.open_slow_btn, self.open_fast_btn, self.open_manual_btn, self.open_plot_btn, self.disconnect_btn]:
            btn.config(state="disabled")
        self.connect_btn.config(state="normal")
        messagebox.showinfo("Success", "All hardware disconnected.")
        
    def _find_hardware(self, is_sim):
        if is_sim: return "SIM_KQD", "SIM_KM"
        from usbfinder import USBTTYFinder
        usb = USBTTYFinder(); kqd_port, km_port = None, None
        try: kqd_port = usb.find_by_product("Position Aligner")[0]
        except Exception: print("WARNING: KQD not found.")
        try: km_port = usb.find_by_product("Brushed Motor Controller")[0]
        except Exception: print("WARNING: Kinesis Motor not found.")
        if kqd_port is None and km_port is None: raise ConnectionError("No alignment hardware found.")
        return kqd_port, km_port

    def _poll_fast_readings(self):
        start_time = time.time()
        while self.fast_polling_active:
            try:
                now = time.time()
                if now - self.last_fast_update_time > self.gui_update_interval:
                    with self.aligner.kqd_lock:
                        if not self.fast_polling_active: break
                        r = self.aligner.kqd.get_readings()
                        mode = self.aligner.kqd.get_operation_mode()
                    if r:
                        self.last_fast_update_time = now
                        status_text = f"Status: {mode.upper()} | X: {r.xdiff:.4f} | Y: {r.ydiff:.4f} | Sum: {r.sum:.4f}"
                        self.after(0, self.fast_status_var.set, status_text)
                        self.after(0, self.fast_mode_var.set, mode)
                        if self.plotting_window and self.plotting_window.winfo_exists():
                            self.plotting_window.update_plot(now - start_time, r.xdiff, r.ydiff)
            except Exception as e:
                print(f"Fast polling error: {e}")
            time.sleep(0.1)
    
    def _apply_fast_mode(self):
        try:
            self._stop_slow_loop() 
            if not self.fast_polling_active:
                self.fast_polling_active = True
                threading.Thread(target=self._poll_fast_readings, daemon=True).start()
            
            mode = self.fast_mode_var.get()
            with self.aligner.kqd_lock:
                if mode == "open_loop":
                    vx, vy = float(self.fast_vx.get()), float(self.fast_vy.get())
                    self.aligner.kqd.set_manual_output(xpos=vx, ypos=vy)
                elif mode == "closed_loop":
                    p, i, d = float(self.fast_p.get()), float(self.fast_i.get()), float(self.fast_d.get())
                    self.aligner.kqd.set_pid_parameters(p=p, i=i, d=d)
                self.aligner.kqd.set_operation_mode(mode)
            print(f"Fast steering mode set to '{mode}'")
        except Exception as e:
            messagebox.showerror("Error", f"Could not set mode: {e}")

    def _get_fast_settings(self):
        try:
            with self.aligner.kqd_lock:
                pid, params = self.aligner.get_fast_steering_params()
            self.fast_p.set(str(pid['p'])); self.fast_i.set(str(pid['i'])); self.fast_d.set(str(pid['d']))
            self.fast_xgain.set(str(params['xgain'])); self.fast_ygain.set(str(params['ygain']))
            self.fast_xmin.set(str(params['xmin'])); self.fast_xmax.set(str(params['xmax']))
            self.fast_ymin.set(str(params['ymin'])); self.fast_ymax.set(str(params['ymax']))
            messagebox.showinfo("Success", "Current fast steering settings loaded.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not get fast settings: {e}")

    def _apply_fast_settings(self):
        try:
            pid = {'p': float(self.fast_p.get()), 'i': float(self.fast_i.get()), 'd': float(self.fast_d.get())}
            params = {
                'xgain': float(self.fast_xgain.get()), 'ygain': float(self.fast_ygain.get()),
                'xmin': float(self.fast_xmin.get()), 'xmax': float(self.fast_xmax.get()),
                'ymin': float(self.fast_ymin.get()), 'ymax': float(self.fast_ymax.get())
            }
            with self.aligner.kqd_lock:
                self.aligner.set_fast_steering_params(pid, params)
            messagebox.showinfo("Success", "Fast steering settings applied.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not apply fast settings: {e}")

    def _run_in_thread(self, target_func, on_complete):
        def task_wrapper():
            try:
                target_func(); self.after(0, on_complete, None)
            except Exception as e:
                self.after(0, on_complete, e)
        threading.Thread(target=task_wrapper, daemon=True).start()

    def _run_axis_calibration(self, axis, btn):
        cal_func = self.aligner.calibrate_x_axis if axis == 'x' else self.aligner.calibrate_y_axis
        btn.config(state="disabled")
        messagebox.showinfo("Calibration", f"Starting {axis.upper()}-axis calibration...")
        def on_complete(error):
            btn.config(state="normal")
            if error: messagebox.showerror("Error", f"{axis.upper()}-axis calibration failed: {error}")
            else:
                messagebox.showinfo("Success", f"{axis.upper()}-axis calibration complete.")
                if self.aligner.Minv is not None: messagebox.showinfo("Success", "Both axes calibrated. Final matrix calculated.")
        self._run_in_thread(cal_func, on_complete)

    def _recalibrate_sum(self):
        try:
            self.aligner.recalibrate_sum_baseline()
            messagebox.showinfo("Success", "Signal baseline recalibrated.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to recalibrate: {e}")

    def _start_slow_loop(self):
        if self.loop_thread and self.loop_thread.is_alive(): messagebox.showwarning("Warning", "Loop is already running."); return
        self._stop_fast_polling()
        try:
            gain_x, gain_y = float(self.loop_gain_x_var.get()), float(self.loop_gain_y_var.get())
        except ValueError:
            messagebox.showerror("Input Error", "Gains must be valid numbers."); return
        if self.plotting_window and self.plotting_window.winfo_exists(): self.plotting_window._clear_plot_data()
        self.start_time = time.time()
        self.aligner.stop_loop_event.clear()
        self.loop_thread = threading.Thread(target=self._run_loop_thread, args=(gain_x, gain_y, self._update_gui_live_data), daemon=True)
        self.loop_thread.start()
        if self.slow_steering_window: self.slow_steering_window.start_loop_btn.config(state="disabled"); self.slow_steering_window.stop_loop_btn.config(state="normal")

    def _run_loop_thread(self, gain_x, gain_y, callback):
        status = self.aligner.run_slow_feedback_loop(loop_gain_x=gain_x, loop_gain_y=gain_y, status_callback=callback)
        status_map = {"stopped_beam_lost": "Stopped: Beam Lost", "stopped_by_user": "Stopped: User Interrupt", "error_not_calibrated": "Error: Not Calibrated", "error_no_baseline": "Error: No Baseline"}
        self.after(0, self._update_loop_status_on_stop, status_map.get(status, "Idle"))
    
    def _update_loop_status_on_stop(self, status):
        self.loop_status_var.set(f"Status: {status}")
        if self.slow_steering_window: self.slow_steering_window.start_loop_btn.config(state="normal"); self.slow_steering_window.stop_loop_btn.config(state="disabled")

    def _stop_slow_loop(self):
        if self.loop_thread and self.loop_thread.is_alive():
            self.aligner.stop_loop_event.set()
            
    def _stop_fast_polling(self):
        if self.fast_polling_active:
            print("Pausing fast steering polling.")
            self.fast_polling_active = False
            time.sleep(0.2)

    def _manual_move(self, axis, direction, var):
        try:
            distance = int(var.get()) * direction
            if axis == 'x': self.aligner.manual_x(distance)
            elif axis == 'y': self.aligner.manual_y(distance)
        except (ValueError, Exception) as e:
            messagebox.showerror("Error", f"Failed to move: {e}")

    def _save_calibration(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")], title="Save Calibration File")
        if not filepath: return
        try:
            self.aligner.save_calibration_to_file(filepath); messagebox.showinfo("Success", "Calibration data saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save data: {e}")

    def _load_calibration(self):
        filepath = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")], title="Load Calibration File")
        if not filepath: return
        try:
            self.aligner.load_calibration_from_file(filepath); messagebox.showinfo("Success", "Calibration data loaded.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load data: {e}")

    def _clear_calibration(self):
        if messagebox.askyesno("Confirm", "Are you sure you want to clear all calibration data?"):
            self.aligner.clear_calibration_data(); messagebox.showinfo("Success", "Calibration data cleared.")
            
    def _on_closing(self):
        if self.is_connected: self._disconnect_all()
        self.destroy()
        
    def _get_slow_settings(self):
        try:
            pid, tic, km = self.aligner.get_pid_settings(), self.aligner.get_tic_settings(), self.aligner.get_km_settings()
            self.pid_x_p.set(str(pid['x']['p'])); self.pid_x_i.set(str(pid['x']['i'])); self.pid_x_d.set(str(pid['x']['d']))
            self.pid_y_p.set(str(pid['y']['p'])); self.pid_y_i.set(str(pid['y']['i'])); self.pid_y_d.set(str(pid['y']['d']))
            self.tic_vel.set(str(tic['velocity'])); self.km_vel.set(str(km['velocity']))
            self.tic_current.set(str(tic['current_limit'])); self.tic_step_str.set(self.inv_step_mode_map.get(tic['step_mode'], "Unknown"))
            messagebox.showinfo("Success", "Current settings loaded.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not get settings: {e}")
            
    def _apply_slow_settings(self):
        try:
            self.aligner.set_pid_settings(float(self.pid_x_p.get()), float(self.pid_x_i.get()), float(self.pid_x_d.get()), float(self.pid_y_p.get()), float(self.pid_y_i.get()), float(self.pid_y_d.get()))
            step_mode_int = self.step_mode_map.get(self.tic_step_str.get())
            self.aligner.set_tic_settings(int(self.tic_vel.get()), int(self.tic_current.get()), step_mode_int)
            self.aligner.set_km_settings(float(self.km_vel.get()))
            messagebox.showinfo("Success", "Settings applied successfully.")
        except Exception as e:
            messagebox.showerror("Error", f"Could not apply settings: {e}")
            
    def _update_gui_live_data(self, x_err, y_err, current_sum, d_tic, d_gon):
        now = time.time()
        if now - getattr(self, 'last_slow_update_time', 0) < self.gui_update_interval:
            return
        self.last_slow_update_time = now
        elapsed_time = now - self.start_time
        if self.plotting_window and self.plotting_window.winfo_exists():
            self.plotting_window.update_plot(elapsed_time, x_err, y_err)
        status_text = f"Status: Running... | Err:[{x_err:.4f},{y_err:.4f}] | Sum:{current_sum:.3f} | Corr:[{d_tic:.2f},{d_gon:.2f}]"
        self.loop_status_var.set(status_text)
        

if __name__ == "__main__":
    if startup_error:
        print("--- STARTUP FAILED ---"); print(f"Error details: {startup_error}")
        try:
            root = tk.Tk(); root.withdraw()
            messagebox.showerror("Hardware Not Found", f"Could not find required hardware on startup.\n\nDetails: {startup_error}")
        except tk.TclError: pass
    else:
        try:
            app = App()
            app.mainloop()
        except tk.TclError as e:
            print("\n--- GUI FAILED TO START ---")
            print("Could not connect to a graphical display.")
            print("If running over SSH, connect with 'ssh -X pi@<your_pi_ip>'")
            print(f"Error details: {e}")
        except Exception as e:
            print("\n--- UNEXPECTED STARTUP ERROR ---")
            import traceback
            traceback.print_exc()

