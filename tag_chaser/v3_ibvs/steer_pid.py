"""
steer_pid.py -- Discrete PID controller for steering.

Output is clamped to output_limits.  Anti-windup via conditional integration:
the integrator does not accumulate when the output is already saturated and
the new error would push it further into saturation.
"""


class PID:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limits: tuple = (-20.0, 20.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._min, self._max = output_limits
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3

        # Proportional
        p = self.kp * error

        # Derivative (backward difference)
        d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Integrator with anti-windup: only accumulate if not saturated,
        # or if the new error would reduce the integral magnitude.
        raw = p + self.ki * self._integral + d
        if raw >= self._max and error > 0:
            pass   # saturated high — don't integrate further
        elif raw <= self._min and error < 0:
            pass   # saturated low — don't integrate further
        else:
            self._integral += error * dt

        output = p + self.ki * self._integral + d
        return max(self._min, min(self._max, output))

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
