"""Turn-then-drive controller for one cell-to-cell move.

It knows nothing about simulators or ROS: poses go in, a body velocity comes
out. Every differential-drive backend uses it, so the motion tested in
simulation is the motion the real robots perform.
"""
import math


def wheel_speeds(v: float, w: float, wheel_radius: float,
                 wheel_separation: float) -> tuple[float, float]:
    """(left, right) wheel speeds in rad/s for body speed v (m/s) and turn rate w (rad/s)."""
    half = w * wheel_separation / 2
    return (v - half) / wheel_radius, (v + half) / wheel_radius


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class CellController:
    """Turn on the spot towards a target point, then drive straight to it."""

    def __init__(self, control: dict, cruise_lin_vel: float, cruise_ang_vel: float):
        """`control` holds kp_ang, kp_lin, pos_tol (m) and ang_tol (rad)."""
        self._control = control
        self._cruise_lin_vel = cruise_lin_vel
        self._cruise_ang_vel = cruise_ang_vel
        self._target = None
        self._turning = False

    @property
    def arrived(self) -> bool:
        return self._target is None

    def go_to(self, target: tuple[float, float]) -> None:
        self._target = target
        self._turning = True

    def stop(self) -> None:
        self._target = None

    def command(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        """Body velocity (v, w) for the current pose. (0, 0) once the target is reached."""
        if self._target is None:
            return 0.0, 0.0
        dx, dy = self._target[0] - x, self._target[1] - y
        distance = math.hypot(dx, dy)
        if distance < self._control["pos_tol"]:
            self._target = None
            return 0.0, 0.0
        error = math.atan2(dy, dx) - yaw
        error = math.atan2(math.sin(error), math.cos(error))  # wrap to [-pi, pi]
        if self._turning and abs(error) < self._control["ang_tol"]:
            self._turning = False
        v = 0.0 if self._turning else min(self._cruise_lin_vel, self._control["kp_lin"] * distance)
        w = _clamp(self._control["kp_ang"] * error, self._cruise_ang_vel)
        return v, w
