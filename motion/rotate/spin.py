"""In-place rotation (spin): change heading without changing position.

The chassis pivots around its own centre by driving the two wheels at the
same magnitude but opposite directions. With wheel-base ``L`` and wheel
linear speed ``v`` (cm/s), the chassis angular speed is::

    omega_rad_per_s = 2 * v / L

For "angle" mode we integrate the two wheels' travelled distances::

    chassis_angle_rad = (|d_left| + |d_right|) / L
"""

from __future__ import annotations

import math
import time

from . import _hw
from ._hw import (
    LEFT,
    RIGHT,
    MIN_START_DUTY,
    WHEEL_BASE_CM,
    WheelSpeedPID,
)


# Control-loop period for closed-loop speed regulation, seconds.
_PID_PERIOD_S = 0.1
# Window used to estimate wheel speed inside the PID loop, seconds.
_SPEED_WINDOW_S = 0.1


def _wheel_speed_for_omega(omega_deg_per_s: float) -> float:
    """Wheel linear speed (cm/s) needed for a target chassis angular speed.

    Args:
        omega_deg_per_s: chassis angular speed in degrees per second.
    """
    omega_rad = math.radians(omega_deg_per_s)
    return abs(omega_rad) * WHEEL_BASE_CM / 2.0


def spin(direction: int,
         mode: str,
         *,
         angular_speed_dps: float = 90.0,
         angle_deg: float | None = None,
         duration_s: float | None = None) -> None:
    """Rotate the chassis in place.

    The robot's position is preserved (the two wheels turn in opposite
    directions); only its heading changes.

    Args:
        direction: :data:`motion.rotate.LEFT` to spin counter-clockwise (left
            wheel back, right wheel forward), :data:`motion.rotate.RIGHT` to
            spin clockwise.
        mode: ``"speed"`` runs continuously at ``angular_speed_dps`` (for at
            most ``duration_s`` if given, else until interrupted).
            ``"angle"`` rotates exactly ``angle_deg`` and then stops, using
            encoder feedback.
        angular_speed_dps: target chassis angular speed, degrees / second.
            Used in both modes (in ``"angle"`` mode it sets the cruise
            speed during the rotation).
        angle_deg: required in ``"angle"`` mode. Total angle to rotate, in
            degrees. Must be positive; the sign is given by ``direction``.
        duration_s: optional cap for ``"speed"`` mode. ``None`` means run
            until ``KeyboardInterrupt``.

    Raises:
        ValueError: if arguments are inconsistent with the chosen mode.
        RuntimeError: if :func:`motion.rotate.init_rotate` was not called.
    """
    _hw.require_initialised()
    if direction not in (LEFT, RIGHT):
        raise ValueError(f"direction must be LEFT or RIGHT, got {direction!r}")
    if angular_speed_dps <= 0:
        raise ValueError("angular_speed_dps must be positive")

    # LEFT spin: right wheel forward, left wheel reverse.
    # RIGHT spin: right wheel reverse, left wheel forward.
    if direction == LEFT:
        _hw.set_wheel_direction(RIGHT, forward=True)
        _hw.set_wheel_direction(LEFT, forward=False)
    else:
        _hw.set_wheel_direction(RIGHT, forward=False)
        _hw.set_wheel_direction(LEFT, forward=True)

    target_v = _wheel_speed_for_omega(angular_speed_dps)
    init_duty = _hw.cmps_to_duty_estimate(target_v)

    if mode == "speed":
        _spin_speed_mode(target_v, init_duty, duration_s)
    elif mode == "angle":
        if angle_deg is None or angle_deg <= 0:
            raise ValueError("angle_deg must be a positive number in 'angle' mode")
        _spin_angle_mode(target_v, init_duty, angle_deg)
    else:
        raise ValueError(f"mode must be 'speed' or 'angle', got {mode!r}")

    _hw.stop_all()


def _spin_speed_mode(target_v: float,
                     init_duty: float,
                     duration_s: float | None) -> None:
    """Closed-loop constant-angular-speed spin."""
    pid_l = WheelSpeedPID(target_v, init_duty)
    pid_r = WheelSpeedPID(target_v, init_duty)

    _hw.set_duty(LEFT, init_duty)
    _hw.set_duty(RIGHT, init_duty)

    t0 = time.time()
    try:
        while True:
            v_l, v_r = _hw.sample_speeds(_SPEED_WINDOW_S)
            _hw.set_duty(LEFT, pid_l.update(v_l))
            _hw.set_duty(RIGHT, pid_r.update(v_r))
            if duration_s is not None and time.time() - t0 >= duration_s:
                return
            time.sleep(max(0.0, _PID_PERIOD_S - _SPEED_WINDOW_S))
    except KeyboardInterrupt:
        return


def _spin_angle_mode(target_v: float,
                     init_duty: float,
                     angle_deg: float) -> None:
    """Closed-loop spin by a fixed angle (encoder-integrated).

    Stops once the integrated chassis angle reaches ``angle_deg``.
    """
    target_rad = math.radians(angle_deg)

    pid_l = WheelSpeedPID(target_v, init_duty)
    pid_r = WheelSpeedPID(target_v, init_duty)

    _hw.reset_encoders()
    _hw.set_duty(LEFT, init_duty)
    _hw.set_duty(RIGHT, init_duty)

    try:
        while True:
            t_start = time.time()
            v_l, v_r = _hw.sample_speeds(_SPEED_WINDOW_S)

            l_pulses, r_pulses = _hw.get_pulse_counts()
            d_l = _hw.pulses_to_cm(l_pulses)
            d_r = _hw.pulses_to_cm(r_pulses)
            chassis_rad = (d_l + d_r) / WHEEL_BASE_CM
            if chassis_rad >= target_rad:
                return

            # Slow down on approach: drop the setpoint linearly in the last
            # 15 degrees so the robot doesn't overshoot when it stops.
            remaining_rad = target_rad - chassis_rad
            slow_zone = math.radians(15.0)
            if remaining_rad < slow_zone:
                scale = max(0.3, remaining_rad / slow_zone)
                pid_l.retarget(target_v * scale)
                pid_r.retarget(target_v * scale)

            _hw.set_duty(LEFT, pid_l.update(v_l))
            _hw.set_duty(RIGHT, pid_r.update(v_r))

            elapsed = time.time() - t_start
            time.sleep(max(0.0, _PID_PERIOD_S - elapsed))
    except KeyboardInterrupt:
        return
