"""Circular orbit: revolve around an external point while keeping the
chassis heading tangent to the circle.

Geometry
--------
Let ``R`` be the desired orbit radius (centre of the circle to centre of the
chassis), and ``L`` the wheel base. Because the chassis heading is tangent
to the circle, the orbit centre lies perpendicular to the current heading,
at distance ``R``, to the **left** of the chassis for CCW orbits and to the
**right** for CW orbits.

The inner wheel traces a circle of radius ``R - L/2`` and the outer wheel
``R + L/2``. Both wheels move forward, so their linear-speed ratio is::

    v_outer / v_inner = (R + L/2) / (R - L/2)

Given a desired chassis tangential speed ``v_centre`` (cm/s along the
centre's path), each wheel's target speed is::

    v_inner  = v_centre * (R - L/2) / R
    v_outer  = v_centre * (R + L/2) / R

The angle swept around the orbit centre is the same for both wheels::

    theta_rad = arc_centre / R = (d_inner + d_outer) / (2 * R)
"""

from __future__ import annotations

import math
import time

from . import _hw
from ._hw import (
    CW,
    CCW,
    LEFT,
    RIGHT,
    WHEEL_BASE_CM,
    WheelSpeedPID,
)


_PID_PERIOD_S = 0.1
_SPEED_WINDOW_S = 0.1


def _wheel_targets(radius_cm: float, v_centre_cmps: float) -> tuple[float, float]:
    """Inner / outer wheel target linear speeds for a given centre speed.

    Args:
        radius_cm: orbit radius measured from the orbit centre to the
            chassis centre, in cm. Must be strictly greater than
            ``WHEEL_BASE_CM / 2`` so the inner wheel still rolls forward.
        v_centre_cmps: tangential speed of the chassis centre along the
            orbit, cm/s.

    Returns:
        ``(v_inner_cmps, v_outer_cmps)`` wheel speed targets.
    """
    half_base = WHEEL_BASE_CM / 2.0
    if radius_cm <= half_base:
        raise ValueError(
            f"radius_cm must be > wheel_base/2 ({half_base} cm), "
            f"got {radius_cm}"
        )
    v_inner = v_centre_cmps * (radius_cm - half_base) / radius_cm
    v_outer = v_centre_cmps * (radius_cm + half_base) / radius_cm
    return v_inner, v_outer


def orbit(direction: str,
          radius_cm: float,
          mode: str,
          *,
          speed_cmps: float = 10.0,
          angle_deg: float | None = None,
          duration_s: float | None = None) -> None:
    """Revolve around a point while keeping the current heading as tangent.

    Args:
        direction: :data:`motion.rotate.CW` (clockwise, orbit centre is to
            the chassis's right) or :data:`motion.rotate.CCW`
            (counter-clockwise, orbit centre is to the chassis's left).
        radius_cm: orbit radius (chassis centre to orbit centre) in cm.
            Must exceed half the wheel base.
        mode: ``"speed"`` runs continuously at ``speed_cmps`` (capped by
            ``duration_s`` if given). ``"angle"`` orbits exactly
            ``angle_deg`` around the orbit centre and then stops.
        speed_cmps: tangential speed of the chassis centre, cm/s. Used in
            both modes.
        angle_deg: required in ``"angle"`` mode, in degrees around the
            orbit centre (positive). The sign is given by ``direction``.
        duration_s: optional cap for ``"speed"`` mode.

    Raises:
        ValueError: if arguments are inconsistent with the chosen mode.
        RuntimeError: if :func:`motion.rotate.init_rotate` was not called.
    """
    _hw.require_initialised()
    if direction not in (CW, CCW):
        raise ValueError(f"direction must be CW or CCW, got {direction!r}")
    if speed_cmps <= 0:
        raise ValueError("speed_cmps must be positive")

    v_inner, v_outer = _wheel_targets(radius_cm, speed_cmps)

    # Map "inner / outer" onto the physical left / right wheels. Both wheels
    # drive forward for an orbit (no reversing), the difference is purely in
    # PWM magnitude.
    #   CCW: orbit centre on the left  -> left wheel = inner (slower)
    #   CW : orbit centre on the right -> right wheel = inner (slower)
    if direction == CCW:
        v_left, v_right = v_inner, v_outer
    else:
        v_left, v_right = v_outer, v_inner

    _hw.set_wheel_direction(LEFT, forward=True)
    _hw.set_wheel_direction(RIGHT, forward=True)

    duty_l = _hw.cmps_to_duty_estimate(v_left)
    duty_r = _hw.cmps_to_duty_estimate(v_right)

    pid_l = WheelSpeedPID(v_left, duty_l)
    pid_r = WheelSpeedPID(v_right, duty_r)

    if mode == "speed":
        _orbit_speed_mode(pid_l, pid_r, duty_l, duty_r, duration_s)
    elif mode == "angle":
        if angle_deg is None or angle_deg <= 0:
            raise ValueError("angle_deg must be a positive number in 'angle' mode")
        _orbit_angle_mode(pid_l, pid_r, duty_l, duty_r,
                          v_left, v_right, radius_cm, angle_deg)
    else:
        raise ValueError(f"mode must be 'speed' or 'angle', got {mode!r}")

    _hw.stop_all()


def _orbit_speed_mode(pid_l: WheelSpeedPID,
                      pid_r: WheelSpeedPID,
                      duty_l: float,
                      duty_r: float,
                      duration_s: float | None) -> None:
    """Closed-loop constant-speed orbit."""
    _hw.set_duty(LEFT, duty_l)
    _hw.set_duty(RIGHT, duty_r)

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


def _orbit_angle_mode(pid_l: WheelSpeedPID,
                      pid_r: WheelSpeedPID,
                      duty_l: float,
                      duty_r: float,
                      v_left_target: float,
                      v_right_target: float,
                      radius_cm: float,
                      angle_deg: float) -> None:
    """Closed-loop orbit by a fixed swept angle (encoder-integrated)."""
    target_rad = math.radians(angle_deg)

    _hw.reset_encoders()
    _hw.set_duty(LEFT, duty_l)
    _hw.set_duty(RIGHT, duty_r)

    try:
        while True:
            t_start = time.time()
            v_l, v_r = _hw.sample_speeds(_SPEED_WINDOW_S)

            l_pulses, r_pulses = _hw.get_pulse_counts()
            d_l = _hw.pulses_to_cm(l_pulses)
            d_r = _hw.pulses_to_cm(r_pulses)
            theta_rad = (d_l + d_r) / (2.0 * radius_cm)
            if theta_rad >= target_rad:
                return

            # Approach slowdown — same trick as spin's angle mode.
            remaining_rad = target_rad - theta_rad
            slow_zone = math.radians(15.0)
            if remaining_rad < slow_zone:
                scale = max(0.3, remaining_rad / slow_zone)
                pid_l.retarget(v_left_target * scale)
                pid_r.retarget(v_right_target * scale)

            _hw.set_duty(LEFT, pid_l.update(v_l))
            _hw.set_duty(RIGHT, pid_r.update(v_r))

            elapsed = time.time() - t_start
            time.sleep(max(0.0, _PID_PERIOD_S - elapsed))
    except KeyboardInterrupt:
        return
