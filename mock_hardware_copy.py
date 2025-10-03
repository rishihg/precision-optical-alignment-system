import time
import collections
import random
import numpy as np

class MockKQD:
    """
    A mock KinesisQuadDetector for simulation with realistic behavior.
    Simulates actual feedback control and mode effects.
    """
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock KQD.")
        self._is_opened = True
        self._mode = "monitor"
        
        # Named tuples for return types
        self.Readings = collections.namedtuple('Readings', ['xdiff', 'ydiff', 'sum'])
        self.PIDParams = collections.namedtuple('PIDParams', ['p', 'i', 'd'])
        self.OutputParams = collections.namedtuple('OutputParams', 
            ['xmin', 'xmax', 'ymin', 'ymax', 'xgain', 'ygain', 'route', 'open_loop_out'])
        
        # PID parameters
        self._pid = {'p': 1.0, 'i': 0.0, 'd': 0.0}
        
        # Output parameters
        self._output = {
            'xmin': -10.0, 'xmax': 10.0, 'ymin': -10.0, 'ymax': 10.0,
            'xgain': 1.0, 'ygain': 1.0, 'route': 'sma_only', 'open_loop_out': 'zero'
        }
        
        # Simulated beam state
        self._true_beam_x = 0.0  # True beam position
        self._true_beam_y = 0.0
        self._beam_drift_x = 0.0  # Accumulated drift
        self._beam_drift_y = 0.0
        self._manual_x = 0.0  # Manual output voltage
        self._manual_y = 0.0
        
        # PID state for closed loop
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._last_error_x = 0.0
        self._last_error_y = 0.0
        self._last_update_time = time.time()
        
        # Noise and drift parameters
        self._noise_level = 0.005  # Sensor noise
        self._drift_rate = 0.0002  # Drift per second

    def get_readings(self):
        """Get simulated quad detector readings with realistic behavior."""
        current_time = time.time()
        dt = current_time - self._last_update_time
        self._last_update_time = current_time
        
        # Add slow drift to beam
        self._beam_drift_x += self._drift_rate * dt * random.uniform(-1, 1)
        self._beam_drift_y += self._drift_rate * dt * random.uniform(-1, 1)
        
        # Calculate true beam position (drift + corrections)
        true_x = self._beam_drift_x
        true_y = self._beam_drift_y
        
        # Apply mode-specific corrections
        if self._mode == "open_loop":
            # Manual output directly affects measured position
            true_x -= self._manual_x * self._output['xgain'] * 0.1
            true_y -= self._manual_y * self._output['ygain'] * 0.1
            
        elif self._mode == "closed_loop":
            # PID feedback correction
            error_x = true_x
            error_y = true_y
            
            # Calculate PID output
            self._integral_x += error_x * dt
            self._integral_y += error_y * dt
            
            derivative_x = (error_x - self._last_error_x) / dt if dt > 0 else 0
            derivative_y = (error_y - self._last_error_y) / dt if dt > 0 else 0
            
            correction_x = (self._pid['p'] * error_x + 
                          self._pid['i'] * self._integral_x + 
                          self._pid['d'] * derivative_x)
            correction_y = (self._pid['p'] * error_y + 
                          self._pid['i'] * self._integral_y + 
                          self._pid['d'] * derivative_y)
            
            # Apply gain limits
            correction_x = np.clip(correction_x, -1.0, 1.0)
            correction_y = np.clip(correction_y, -1.0, 1.0)
            
            # Corrections reduce the error
            true_x -= correction_x * self._output['xgain'] * 0.1
            true_y -= correction_y * self._output['ygain'] * 0.1
            
            self._last_error_x = error_x
            self._last_error_y = error_y
        
        # Add sensor noise
        measured_x = true_x + random.gauss(0, self._noise_level)
        measured_y = true_y + random.gauss(0, self._noise_level)
        
        # Simulate sum signal (total intensity) with slight variation
        base_sum = 0.95
        sum_signal = base_sum + 0.05 * random.uniform(-1, 1)
        
        return self.Readings(xdiff=measured_x, ydiff=measured_y, sum=sum_signal)

    def get_operation_mode(self):
        return self._mode
    
    def set_operation_mode(self, mode):
        print(f"SIM: KQD mode set to '{mode}'")
        if mode in ["monitor", "open_loop", "closed_loop"]:
            self._mode = mode
            # Reset PID state when changing modes
            if mode == "closed_loop":
                self._integral_x = 0.0
                self._integral_y = 0.0
                self._last_error_x = 0.0
                self._last_error_y = 0.0
        else:
            print(f"WARNING: Unknown mode '{mode}', staying in current mode")
    
    def set_pid_parameters(self, p=None, i=None, d=None, **kwargs):
        if p is not None: self._pid['p'] = p
        if i is not None: self._pid['i'] = i
        if d is not None: self._pid['d'] = d
        print(f"SIM: KQD PID parameters set to P={self._pid['p']}, I={self._pid['i']}, D={self._pid['d']}")
    
    def get_pid_parameters(self):
        return self.PIDParams(p=self._pid['p'], i=self._pid['i'], d=self._pid['d'])
    
    def set_manual_output(self, xpos=None, ypos=None, **kwargs):
        if xpos is not None: self._manual_x = xpos
        if ypos is not None: self._manual_y = ypos
        print(f"SIM: KQD manual output set to X={self._manual_x}, Y={self._manual_y}")
    
    def get_output_parameters(self):
        return self.OutputParams(**self._output)
    
    def set_output_parameters(self, **kwargs):
        # Update only provided parameters
        for key, value in kwargs.items():
            if key in self._output:
                self._output[key] = value
        print(f"SIM: KQD output parameters updated: {kwargs}")
    
    def is_opened(self):
        return self._is_opened
    
    def close(self):
        self._is_opened = False
        print("SIM: KQD connection closed.")


class MockKinesisMotor:
    """
    A mock KinesisMotor for simulation with realistic motion.
    """
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock Kinesis Motor.")
        self._position = 50000  # Start at mid-range
        self._target_position = 50000
        self._is_opened = True
        self._is_moving = False
        
        self.VelocityParameters = collections.namedtuple('VelocityParameters', 
            ['min_velocity', 'acceleration', 'max_velocity'])
        self._velocity = 2.0
        self._acceleration = 1.0
        
        # Motion simulation
        self._last_update_time = time.time()
        self._motion_thread = None

    def get_position(self):
        # Simulate gradual motion toward target
        if self._is_moving:
            current_time = time.time()
            dt = current_time - self._last_update_time
            self._last_update_time = current_time
            
            # Calculate distance to target
            distance = self._target_position - self._position
            max_step = self._velocity * dt * 10000  # Scale factor for simulation
            
            if abs(distance) < max_step:
                self._position = self._target_position
                self._is_moving = False
            else:
                self._position += np.sign(distance) * max_step
        
        return int(self._position)
    
    def move_to(self, pos):
        self._target_position = pos
        self._is_moving = True
        self._last_update_time = time.time()
        print(f"SIM: Kinesis motor moving to {pos}")
        # Don't sleep here - let get_position() simulate gradual motion
    
    def move_by(self, dist):
        current = self.get_position()
        self.move_to(current + dist)
    
    def wait_move(self):
        # Simulate blocking until motion complete
        while self._is_moving:
            self.get_position()  # Update position
            time.sleep(0.01)
    
    def stop(self, *args, **kwargs):
        self._is_moving = False
        self._target_position = self._position
        print("SIM: Kinesis motor stopped.")
    
    def get_velocity_parameters(self):
        return self.VelocityParameters(
            min_velocity=0, 
            acceleration=self._acceleration, 
            max_velocity=self._velocity
        )
    
    def setup_velocity(self, max_velocity=None, acceleration=None, **kwargs):
        if max_velocity is not None:
            self._velocity = max_velocity
        if acceleration is not None:
            self._acceleration = acceleration
        print(f"SIM: Kinesis motor velocity set to {self._velocity}, accel={self._acceleration}")
    
    def is_opened(self):
        return self._is_opened
    
    def close(self):
        self._is_opened = False
        self._is_moving = False
        print("SIM: Kinesis motor connection closed.")
    
    def set_position_software_limits(self, *args, **kwargs):
        print("SIM: Kinesis motor soft limits set.")
    
    def get_travel_range(self):
        return (0, 100000)


class MockTic:
    """
    A mock TicUSB controller for simulation with realistic stepper behavior.
    """
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock Tic Controller.")
        self._position = 0
        self._target_position = 0
        self._max_speed = 2000000
        self._current_limit = 1000
        self._step_mode = 5
        self._energized = False
        
        # Motion parameters
        self._is_moving = False
        self._last_update_time = time.time()

    def get_current_position(self):
        # Simulate gradual motion
        if self._is_moving and self._energized:
            current_time = time.time()
            dt = current_time - self._last_update_time
            self._last_update_time = current_time
            
            distance = self._target_position - self._position
            max_step = self._max_speed * dt / 1000  # Scale for simulation
            
            if abs(distance) < max_step:
                self._position = self._target_position
                self._is_moving = False
            else:
                self._position += int(np.sign(distance) * max_step)
        
        return int(self._position)
    
    def set_target_position(self, pos):
        if not self._energized:
            print("WARNING: Tic not energized, move ignored")
            return
        
        self._target_position = int(pos)
        self._is_moving = True
        self._last_update_time = time.time()
        # Don't print or sleep for every small move - too verbose
    
    def halt_and_set_position(self, pos):
        self._position = int(pos)
        self._target_position = int(pos)
        self._is_moving = False
        print(f"SIM: Tic position set to {pos}")
    
    def halt_and_hold(self):
        self._is_moving = False
        self._target_position = self._position
        print("SIM: Tic motor halted and holding.")
    
    def energize(self):
        self._energized = True
        print("SIM: Tic energized.")
    
    def exit_safe_start(self):
        print("SIM: Tic exited safe start.")
    
    def deenergize(self):
        self._energized = False
        self._is_moving = False
        print("SIM: Tic deenergized.")
    
    def get_max_speed(self):
        return self._max_speed
    
    def get_current_limit(self):
        return self._current_limit
    
    def get_step_mode(self):
        return self._step_mode
    
    def set_max_speed(self, speed):
        self._max_speed = int(speed)
        print(f"SIM: Tic max speed set to {speed}")
    
    def set_current_limit(self, limit):
        self._current_limit = int(limit)
        print(f"SIM: Tic current limit set to {limit}")
    
    def set_step_mode(self, mode):
        self._step_mode = int(mode)
        print(f"SIM: Tic step mode set to {mode}")
    
    def set_setting(self, *args, **kwargs):
        pass