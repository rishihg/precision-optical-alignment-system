import time

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0.0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None

        self.min_output, self.max_output = output_limits

    def compute(self, measurement, now=None):
        """Compute new control signal based on measurement."""
        if now is None:
            now = time.time()

        error = self.setpoint - measurement

        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time

        # Proportional
        P = self.Kp * error

        # Integral
        self._integral += error * dt
        I = self.Ki * self._integral

        # Derivative
        D = 0.0
        if dt > 0:
            D = self.Kd * (error - self._prev_error) / dt

        # PID sum
        output = P + I + D

        # Clamp output
        if self.min_output is not None:
            output = max(self.min_output, output)
        if self.max_output is not None:
            output = min(self.max_output, output)

        # Save state
        self._prev_error = error
        self._last_time = now

        return output

    def reset(self):
        """Reset PID memory (integral, derivative)."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._last_time = None
