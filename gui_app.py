import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from optical_aligner import OpticalAligner

class App(tk.Tk):
    """
    Main GUI application for the Optical Alignment Control system.
    """
    def __init__(self):
        super().__init__()
        self.title("Optical Alignment Control")
        self.geometry("600x680") # Increased height for status label

        # --- Hardware Backend ---
        self.aligner = OpticalAligner()
        self.slow_steering_connected = False
        self.fast_steering_connected = False

        # --- Threading Control ---
        self.limit_polling_active = False

        # --- UI Setup ---
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(expand=True, fill='both', padx=10, pady=10)

        self._create_fast_steering_tab()
        self._create_slow_steering_tab()
        self._create_manual_control_tab()

        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    # --- Tab Creation Methods ---
    def _create_fast_steering_tab(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Fast Steering')
        conn_frame = ttk.LabelFrame(tab, text="Hardware Connection", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=10)
        ttk.Label(conn_frame, text="Fast steering control is managed via its command-line script.").pack(pady=5)
        self.connect_fast_btn = ttk.Button(conn_frame, text="Run fast_steering.py", command=lambda: messagebox.showinfo("Info", "Please run the 'fast_steering.py' script from your terminal to use this feature."))
        self.connect_fast_btn.pack(pady=5, fill='x')

    def _create_slow_steering_tab(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Slow Steering')

        conn_frame = ttk.LabelFrame(tab, text="Hardware Connection", padding=10)
        conn_frame.pack(fill='x', padx=10, pady=10)
        self.connect_slow_btn = ttk.Button(conn_frame, text="Connect", command=self._connect_slow_steering)
        self.connect_slow_btn.pack(side='left', padx=5)
        self.disconnect_slow_btn = ttk.Button(conn_frame, text="Disconnect", state='disabled', command=self._disconnect_slow_steering)
        self.disconnect_slow_btn.pack(side='left', padx=5)

        control_frame = ttk.LabelFrame(tab, text="Main Controls", padding=10)
        control_frame.pack(fill='x', padx=10, pady=10)

        self.calibrate_btn = ttk.Button(control_frame, text="Run Calibration", state='disabled', command=self._run_calibration)
        self.calibrate_btn.pack(pady=5, fill='x')
        self.recalibrate_sum_btn = ttk.Button(control_frame, text="Recalibrate Baseline", state='disabled', command=self._recalibrate_sum)
        self.recalibrate_sum_btn.pack(pady=5, fill='x')
        
        # --- Loop Control Frame ---
        loop_frame = ttk.Frame(control_frame)
        loop_frame.pack(pady=5, fill='x')
        self.start_loop_btn = ttk.Button(loop_frame, text="Start Slow Loop", state='disabled', command=self._start_slow_loop)
        self.start_loop_btn.pack(side='left', expand=True, fill='x', padx=(0, 5))
        self.stop_loop_btn = ttk.Button(loop_frame, text="Stop Loop", state='disabled', command=self._stop_slow_loop)
        self.stop_loop_btn.pack(side='left', expand=True, fill='x', padx=(5, 0))
        
        self.loop_status_var = tk.StringVar(value="Status: Idle")
        self.loop_status_label = ttk.Label(control_frame, textvariable=self.loop_status_var, font=("TkDefaultFont", 10, "italic"))
        self.loop_status_label.pack(pady=(10, 0))

        self._create_slow_settings_frame(tab)
        
    def _create_manual_control_tab(self):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text='Manual Control')

        x_frame = ttk.LabelFrame(tab, text="X-Axis Control (Tic)", padding=10)
        x_frame.pack(fill='x', padx=10, pady=10)
        ttk.Label(x_frame, text="Magnitude:").pack(side='left', padx=5)
        self.x_move_entry = ttk.Entry(x_frame, width=10)
        self.x_move_entry.pack(side='left', padx=5)
        self.x_move_entry.insert(0, "100")
        self.x_move_neg_btn = ttk.Button(x_frame, text="-X", width=4, state='disabled', command=lambda: self._manual_move('x', -1))
        self.x_move_neg_btn.pack(side='left', padx=5)
        self.x_move_pos_btn = ttk.Button(x_frame, text="+X", width=4, state='disabled', command=lambda: self._manual_move('x', 1))
        self.x_move_pos_btn.pack(side='left', padx=5)
        self.x_limit_indicator = tk.Label(x_frame, text=" ", bg="grey", width=2, relief='sunken')
        self.x_limit_indicator.pack(side='right', padx=10)
        ttk.Label(x_frame, text="Limit:").pack(side='right')

        y_frame = ttk.LabelFrame(tab, text="Y-Axis Control (Goniometer)", padding=10)
        y_frame.pack(fill='x', padx=10, pady=10)
        ttk.Label(y_frame, text="Magnitude:").pack(side='left', padx=5)
        self.y_move_entry = ttk.Entry(y_frame, width=10)
        self.y_move_entry.pack(side='left', padx=5)
        self.y_move_entry.insert(0, "1000")
        self.y_move_neg_btn = ttk.Button(y_frame, text="-Y", width=4, state='disabled', command=lambda: self._manual_move('y', -1))
        self.y_move_neg_btn.pack(side='left', padx=5)
        self.y_move_pos_btn = ttk.Button(y_frame, text="+Y", width=4, state='disabled', command=lambda: self._manual_move('y', 1))
        self.y_move_pos_btn.pack(side='left', padx=5)

        x_limit_frame = ttk.LabelFrame(tab, text="X-Axis Soft Limits", padding=10)
        x_limit_frame.pack(fill='x', padx=10, pady=10)
        ttk.Label(x_limit_frame, text="Range (+/-):").pack(side='left', padx=5)
        self.x_limit_entry = ttk.Entry(x_limit_frame, width=10)
        self.x_limit_entry.pack(side='left', padx=5)
        self.x_limit_entry.insert(0, "500")
        self.set_x_limits_btn = ttk.Button(x_limit_frame, text="Set Limits", state='disabled', command=lambda: self._set_soft_limits('x'))
        self.set_x_limits_btn.pack(side='left', padx=5)
        self.clear_x_limits_btn = ttk.Button(x_limit_frame, text="Clear Limits", state='disabled', command=lambda: self._clear_soft_limits('x'))
        self.clear_x_limits_btn.pack(side='left', padx=5)
        
        y_limit_frame = ttk.LabelFrame(tab, text="Y-Axis Soft Limits", padding=10)
        y_limit_frame.pack(fill='x', padx=10, pady=10)
        ttk.Label(y_limit_frame, text="Range (+/-):").pack(side='left', padx=5)
        self.y_limit_entry = ttk.Entry(y_limit_frame, width=10)
        self.y_limit_entry.pack(side='left', padx=5)
        self.y_limit_entry.insert(0, "2000")
        self.set_y_limits_btn = ttk.Button(y_limit_frame, text="Set Limits", state='disabled', command=lambda: self._set_soft_limits('y'))
        self.set_y_limits_btn.pack(side='left', padx=5)
        self.clear_y_limits_btn = ttk.Button(y_limit_frame, text="Clear Limits", state='disabled', command=lambda: self._clear_soft_limits('y'))
        self.clear_y_limits_btn.pack(side='left', padx=5)

    def _create_slow_settings_frame(self, parent_tab):
        settings_frame = ttk.LabelFrame(parent_tab, text="Settings", padding=10)
        settings_frame.pack(fill='x', padx=10, pady=10)
        settings_frame.columnconfigure(1, weight=1)
        settings_frame.columnconfigure(3, weight=1)

        ttk.Label(settings_frame, text="X PID (P/I/D):").grid(row=0, column=0, sticky='w', pady=2)
        self.pid_x_p = ttk.Entry(settings_frame, width=7)
        self.pid_x_p.grid(row=0, column=1, sticky='ew', padx=2)
        self.pid_x_i = ttk.Entry(settings_frame, width=7)
        self.pid_x_i.grid(row=0, column=2, sticky='ew', padx=2)
        self.pid_x_d = ttk.Entry(settings_frame, width=7)
        self.pid_x_d.grid(row=0, column=3, sticky='ew', padx=2)

        ttk.Label(settings_frame, text="Y PID (P/I/D):").grid(row=1, column=0, sticky='w', pady=2)
        self.pid_y_p = ttk.Entry(settings_frame, width=7)
        self.pid_y_p.grid(row=1, column=1, sticky='ew', padx=2)
        self.pid_y_i = ttk.Entry(settings_frame, width=7)
        self.pid_y_i.grid(row=1, column=2, sticky='ew', padx=2)
        self.pid_y_d = ttk.Entry(settings_frame, width=7)
        self.pid_y_d.grid(row=1, column=3, sticky='ew', padx=2)
        
        ttk.Label(settings_frame, text="X Velocity:").grid(row=2, column=0, sticky='w', pady=2)
        self.tic_vel = ttk.Entry(settings_frame)
        self.tic_vel.grid(row=2, column=1, columnspan=3, sticky='ew', padx=2)
        ttk.Label(settings_frame, text="X Current (mA):").grid(row=3, column=0, sticky='w', pady=2)
        self.tic_curr = ttk.Entry(settings_frame)
        self.tic_curr.grid(row=3, column=1, columnspan=3, sticky='ew', padx=2)
        ttk.Label(settings_frame, text="X Step Mode:").grid(row=4, column=0, sticky='w', pady=2)
        self.tic_step = ttk.Combobox(settings_frame, values=[0, 1, 2, 3, 4, 5], state='readonly')
        self.tic_step.grid(row=4, column=1, columnspan=3, sticky='ew', padx=2)
        
        ttk.Label(settings_frame, text="Y Velocity:").grid(row=5, column=0, sticky='w', pady=2)
        self.km_vel = ttk.Entry(settings_frame)
        self.km_vel.grid(row=5, column=1, columnspan=3, sticky='ew', padx=2)

        btn_frame = ttk.Frame(settings_frame)
        btn_frame.grid(row=6, column=0, columnspan=4, pady=10)
        self.get_settings_btn = ttk.Button(btn_frame, text="Get Current", state='disabled', command=self._get_slow_settings)
        self.get_settings_btn.pack(side='left', padx=5)
        self.apply_settings_btn = ttk.Button(btn_frame, text="Apply Settings", state='disabled', command=self._apply_slow_settings)
        self.apply_settings_btn.pack(side='left', padx=5)

    def _connect_slow_steering(self):
        try:
            self.aligner.kqd_port = "/dev/ttyUSB0"
            self.aligner.km_port = "/dev/ttyUSB1"
            self.aligner.connect_slow_steering()
            self.slow_steering_connected = True
            
            self.connect_slow_btn.config(state='disabled')
            self.disconnect_slow_btn.config(state='normal')
            self.calibrate_btn.config(state='normal')
            self.recalibrate_sum_btn.config(state='normal')
            self.start_loop_btn.config(state='normal')
            self.apply_settings_btn.config(state='normal')
            self.get_settings_btn.config(state='normal')
            self.x_move_pos_btn.config(state='normal')
            self.x_move_neg_btn.config(state='normal')
            self.y_move_pos_btn.config(state='normal')
            self.y_move_neg_btn.config(state='normal')
            self.set_x_limits_btn.config(state='normal')
            self.clear_x_limits_btn.config(state='normal')
            self.set_y_limits_btn.config(state='normal')
            self.clear_y_limits_btn.config(state='normal')

            self.limit_polling_active = True
            self._poll_limit_switches()
            self._get_slow_settings()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")

    def _disconnect_slow_steering(self):
        if self.aligner.stop_loop_event.is_set() is False:
             self.aligner.stop_loop_event.set() # Stop loop if running
        self.limit_polling_active = False
        self.aligner.disconnect_slow_steering()
        self.slow_steering_connected = False
        
        self.connect_slow_btn.config(state='normal')
        self.disconnect_slow_btn.config(state='disabled')
        self.calibrate_btn.config(state='disabled')
        self.recalibrate_sum_btn.config(state='disabled')
        self.start_loop_btn.config(state='disabled')
        self.stop_loop_btn.config(state='disabled')
        self.apply_settings_btn.config(state='disabled')
        self.get_settings_btn.config(state='disabled')
        self.x_move_pos_btn.config(state='disabled')
        self.x_move_neg_btn.config(state='disabled')
        self.y_move_pos_btn.config(state='disabled')
        self.y_move_neg_btn.config(state='disabled')
        self.set_x_limits_btn.config(state='disabled')
        self.clear_x_limits_btn.config(state='disabled')
        self.set_y_limits_btn.config(state='disabled')
        self.clear_y_limits_btn.config(state='disabled')
        self.loop_status_var.set("Status: Disconnected")


    def _run_calibration(self):
        self.calibrate_btn.config(state='disabled', text="Calibrating...")
        self.loop_status_var.set("Status: Calibrating...")
        threading.Thread(target=self._calibration_thread, daemon=True).start()

    def _calibration_thread(self):
        try:
            self.aligner.calibrate()
            self.loop_status_var.set("Status: Calibration successful.")
            messagebox.showinfo("Success", "Calibration completed successfully.")
        except Exception as e:
            self.loop_status_var.set("Status: Calibration FAILED.")
            messagebox.showerror("Calibration Error", f"Calibration failed:\n{e}")
        finally:
            self.calibrate_btn.config(state='normal', text="Run Calibration")

    def _recalibrate_sum(self):
        self.recalibrate_sum_btn.config(state='disabled', text="Recalibrating...")
        self.loop_status_var.set("Status: Recalibrating baseline...")
        threading.Thread(target=self._recalibrate_sum_thread, daemon=True).start()

    def _recalibrate_sum_thread(self):
        try:
            self.aligner.recalibrate_sum_baseline()
            self.loop_status_var.set("Status: Baseline recalibrated.")
            messagebox.showinfo("Success", "Signal baseline recalibrated.")
        except Exception as e:
            self.loop_status_var.set("Status: Baseline recalibration FAILED.")
            messagebox.showerror("Error", f"Could not recalibrate baseline:\n{e}")
        finally:
            self.recalibrate_sum_btn.config(state='normal', text="Recalibrate Baseline")

    def _start_slow_loop(self):
        self.aligner.stop_loop_event.clear()
        self.start_loop_btn.config(state='disabled')
        self.stop_loop_btn.config(state='normal')
        self.loop_status_var.set("Status: Feedback loop RUNNING...")
        threading.Thread(target=self._slow_loop_thread, daemon=True).start()

    def _stop_slow_loop(self):
        self.loop_status_var.set("Status: Stopping loop...")
        self.aligner.stop_loop_event.set()
        self.stop_loop_btn.config(state='disabled')

    def _slow_loop_thread(self):
        try:
            status = self.aligner.run_slow_feedback_loop()
            
            if status == "stopped_beam_lost":
                self.loop_status_var.set("Status: Stopped - Beam signal lost.")
                messagebox.showwarning("Loop Halted", "Feedback loop stopped automatically because the beam signal was lost.")
            elif status == "stopped_by_user":
                 self.loop_status_var.set("Status: Stopped by user.")
            else:
                self.loop_status_var.set("Status: Idle")

        except Exception as e:
            self.loop_status_var.set("Status: Loop ERROR.")
            messagebox.showerror("Feedback Loop Error", f"An error occurred:\n{e}")
        finally:
            self.start_loop_btn.config(state='normal')
            self.stop_loop_btn.config(state='disabled')

    def _get_slow_settings(self):
        try:
            pid = self.aligner.get_pid_settings()
            tic = self.aligner.get_tic_settings()
            km = self.aligner.get_km_settings()

            self.pid_x_p.delete(0, 'end'); self.pid_x_p.insert(0, pid['x']['p'])
            self.pid_x_i.delete(0, 'end'); self.pid_x_i.insert(0, pid['x']['i'])
            self.pid_x_d.delete(0, 'end'); self.pid_x_d.insert(0, pid['x']['d'])
            self.pid_y_p.delete(0, 'end'); self.pid_y_p.insert(0, pid['y']['p'])
            self.pid_y_i.delete(0, 'end'); self.pid_y_i.insert(0, pid['y']['i'])
            self.pid_y_d.delete(0, 'end'); self.pid_y_d.insert(0, pid['y']['d'])
            
            self.tic_vel.delete(0, 'end'); self.tic_vel.insert(0, tic['velocity'])
            self.tic_curr.delete(0, 'end'); self.tic_curr.insert(0, tic['current_limit'])
            self.tic_step.set(tic['step_mode'])
            self.km_vel.delete(0, 'end'); self.km_vel.insert(0, km['velocity'])
            
        except Exception as e:
            messagebox.showerror("Error", f"Could not get settings: {e}")

    def _apply_slow_settings(self):
        try:
            self.aligner.set_pid_settings(
                float(self.pid_x_p.get()), float(self.pid_x_i.get()), float(self.pid_x_d.get()),
                float(self.pid_y_p.get()), float(self.pid_y_i.get()), float(self.pid_y_d.get())
            )
            self.aligner.set_tic_settings(
                int(self.tic_vel.get()), int(self.tic_curr.get()), int(self.tic_step.get())
            )
            self.aligner.set_km_settings(float(self.km_vel.get()))
            messagebox.showinfo("Success", "Settings applied successfully.")
        except ValueError:
            messagebox.showerror("Input Error", "All settings must be valid numbers.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply settings: {e}")

    def _manual_move(self, axis, direction):
        try:
            if axis == 'x':
                magnitude = int(self.x_move_entry.get())
                self.aligner.manual_x(magnitude * direction)
            elif axis == 'y':
                magnitude = int(self.y_move_entry.get())
                self.aligner.manual_y(magnitude * direction)
        except ValueError:
            messagebox.showerror("Input Error", "Magnitude must be an integer.")
        except Exception as e:
            messagebox.showerror("Move Error", f"Failed to move axis: {e}")

    def _set_soft_limits(self, axis):
        try:
            if axis == 'x':
                range_val = int(self.x_limit_entry.get())
                current_pos = self.aligner.tic.get_current_position()
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                self.aligner.set_tic_soft_limits(min_pos, max_pos)
                messagebox.showinfo("Success", f"X-Axis soft limits set to ({min_pos}, {max_pos})")
            elif axis == 'y':
                range_val = int(self.y_limit_entry.get())
                current_pos = self.aligner.km.get_position()
                min_pos, max_pos = current_pos - range_val, current_pos + range_val
                self.aligner.set_km_soft_limits(min_pos, max_pos)
                messagebox.showinfo("Success", f"Y-Axis soft limits set to ({min_pos}, {max_pos})")
        except ValueError:
            messagebox.showerror("Input Error", "Range must be an integer.")
        except Exception as e:
            messagebox.showerror("Limit Error", f"Failed to set limits: {e}")

    def _clear_soft_limits(self, axis):
        try:
            if axis == 'x':
                self.aligner.clear_tic_soft_limits()
                messagebox.showinfo("Success", "X-Axis soft limits cleared.")
            elif axis == 'y':
                self.aligner.clear_km_soft_limits()
                messagebox.showinfo("Success", "Y-Axis soft limits cleared.")
        except Exception as e:
            messagebox.showerror("Limit Error", f"Failed to clear limits: {e}")

    def _poll_limit_switches(self):
        if self.limit_polling_active and self.slow_steering_connected:
            try:
                variables = self.aligner.tic.get_variables()
                limit_active = variables.get('limit_switches', 0)
                if limit_active:
                    self.x_limit_indicator.config(bg="red")
                else:
                    self.x_limit_indicator.config(bg="grey")
            except Exception:
                pass
            self.after(250, self._poll_limit_switches)

    def _on_closing(self):
        self.limit_polling_active = False
        if self.slow_steering_connected:
            self.aligner.stop_loop_event.set() # Stop loop if running
            self.aligner.disconnect_slow_steering()
        if self.fast_steering_connected:
            self.aligner.disconnect_fast_steering()
        self.destroy()

if __name__ == "__main__":
    app = App()
    app.mainloop()

