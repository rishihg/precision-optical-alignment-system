import time
import collections

class MockKQD:
    """A mock KinesisQuadDetector for simulation."""
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock KQD.")
        self._is_opened = True
        self._mode = "monitor"
        self.Readings = collections.namedtuple('Readings', ['xdiff', 'ydiff', 'sum'])

    def get_readings(self):
        import random
        drift = time.time() / 50.0
        return self.Readings(
            xdiff=0.01 * random.uniform(-1, 1) + drift,
            ydiff=0.01 * random.uniform(-1, 1) - drift,
            sum=0.9 + 0.05 * random.uniform(-1, 1)
        )

    def get_operation_mode(self): return self._mode
    def set_operation_mode(self, mode): print(f"SIM: KQD mode set to '{mode}'"); self._mode = mode
    def set_pid_parameters(self, *args, **kwargs): print(f"SIM: KQD PID parameters set.")
    def set_manual_output(self, *args, **kwargs): print(f"SIM: KQD manual output set.")
    def get_output_parameters(self): return {'xmin': -10, 'xmax': 10, 'ymin': -10, 'ymax': 10, 'xgain': 1, 'ygain': 1, 'route': 'sma_only', 'open_loop_out': 'zero'}
    def set_output_parameters(self, *args, **kwargs): print("SIM: KQD output parameters set.")
    def is_opened(self): return self._is_opened
    def close(self): self._is_opened = False; print("SIM: KQD connection closed.")

class MockKinesisMotor:
    """A mock KinesisMotor for simulation."""
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock Kinesis Motor.")
        self._position = 0
        self._is_opened = True
        self.VelocityParameters = collections.namedtuple('VelocityParameters', ['min_velocity', 'acceleration', 'max_velocity'])

    def get_position(self): return self._position
    def move_to(self, pos): print(f"SIM: Kinesis motor moving to {pos}"); self._position = pos; time.sleep(0.1)
    def move_by(self, dist): self.move_to(self._position + dist)
    def wait_move(self): pass
    def stop(self, *args, **kwargs): print("SIM: Kinesis motor stopped.")
    def get_velocity_parameters(self): return self.VelocityParameters(min_velocity=0, acceleration=1.0, max_velocity=2.0)
    def setup_velocity(self, *args, **kwargs): print(f"SIM: Kinesis motor velocity set.")
    def is_opened(self): return self._is_opened
    def close(self): self._is_opened = False; print("SIM: Kinesis motor connection closed.")
    def set_position_software_limits(self, *args, **kwargs): print("SIM: Kinesis motor soft limits set.")
    def get_travel_range(self): return (0, 100000)

class MockTic:
    """A mock TicUSB controller for simulation."""
    def __init__(self, *args, **kwargs):
        print("INFO: Initializing Mock Tic Controller.")
        self._position = 0

    def get_current_position(self): return self._position
    def set_target_position(self, pos): print(f"SIM: Tic motor moving to {pos}"); self._position = pos; time.sleep(0.1)
    def halt_and_set_position(self, pos): self._position = pos
    def halt_and_hold(self): print("SIM: Tic motor halted and holding.")
    def energize(self): pass
    def exit_safe_start(self): pass
    def deenergize(self): pass
    def get_max_speed(self): return 2000000
    def get_current_limit(self): return 1000
    def get_step_mode(self): return 5
    def set_max_speed(self, *args, **kwargs): pass
    def set_current_limit(self, *args, **kwargs): pass
    def set_step_mode(self, *args, **kwargs): pass
    def set_setting(self, *args, **kwargs): pass

