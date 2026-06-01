"""Hardware abstraction shared by spin and orbit.

This module owns:
  * motor direction pin handling (I1/I2 for right wheel, I3/I4 for left wheel),
  * a wheel-encoder based odometer (LS / RS rising-edge counters),
  * physical calibration constants used to convert between PWM duty cycle,
    wheel linear speed and chassis angle.

Pin assignments mirror ``main.py``; calibration constants are placeholders
and MUST be measured on the real chassis before the closed-loop controllers
can be trusted.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import RPi.GPIO as GPIO


# --- Direction enums --------------------------------------------------------

LEFT = 1
RIGHT = 2

CW = "cw"    # clockwise (when viewed from above)
CCW = "ccw"  # counter-clockwise


# --- Pin layout (matches main.py) -------------------------------------------

# Right motor: EA pwm, I1/I2 direction
I1_RIGHT = 26
I2_RIGHT = 19
# Left motor: EB pwm, I3/I4 direction
I3_LEFT = 21
I4_LEFT = 20
# Wheel encoders (from inc_pid.py); main.py does not set these up, so we do.
LS_PIN = 6
RS_PIN = 12


# --- Physical / calibration constants (PLACEHOLDERS — calibrate on hardware)

# Distance between the centres of the two driving wheels, in cm.
# Chassis measurement: 16 cm from left-wheel left-edge to right-wheel
# right-edge, wheel width 2.5 cm  ->  16 - 2*(2.5/2) = 13.5 cm.
WHEEL_BASE_CM = 13.5
# Wheel circumference, in cm (distance per full wheel revolution).
# Wheel radius = 3 cm  ->  2 * pi * 3 ~= 18.8496 cm.
WHEEL_CIRCUMFERENCE_CM = 18.8496
# Encoder pulses per full wheel revolution (inc_pid.py uses 585).
ENCODER_PULSES_PER_REV = 585.0  # TODO: verify
# Approximate linear wheel speed at 100% PWM duty cycle, cm/s.
# Used only for the open-loop initial duty estimate; PID closes the loop.
MAX_LINEAR_SPEED_CMPS = 25.0  # TODO: measure

# Minimum duty cycle the motors will actually start moving at.
MIN_START_DUTY = 18.0
# Conservative cap to keep currents reasonable.
MAX_DUTY = 90.0

# Default closed-loop PID gains for linear-speed control on each wheel.
# Mirrors the structure of inc_pid.py / pid_control.py.
DEFAULT_PID = (4.0, 2.5, 1.2)


# --- Module state -----------------------------------------------------------


@dataclass
class _RotateState:
    """Holds runtime handles. Populated by :func:`init_rotate`."""

    pwm_left: Optional[object] = None
    pwm_right: Optional[object] = None
    l_count: int = 0
    r_count: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)
    initialised: bool = False


_state = _RotateState()


def _on_left_pulse(_channel):
    with _state.lock:
        _state.l_count += 1


def _on_right_pulse(_channel):
    with _state.lock:
        _state.r_count += 1


def init_rotate(pwm_left, pwm_right) -> None:
    """Bind to externally created PWM objects and start encoder counting.

    Assumes ``GPIO.setmode(GPIO.BCM)`` and the EA/EB PWM channels have already
    been configured by the caller (e.g. ``main.py``). The direction pins
    (I1..I4) are re-claimed here so this module fully owns motor direction.

    Args:
        pwm_left: ``RPi.GPIO.PWM`` instance driving the left motor (EB).
        pwm_right: ``RPi.GPIO.PWM`` instance driving the right motor (EA).
    """
    _state.pwm_left = pwm_left
    _state.pwm_right = pwm_right

    GPIO.setup([I1_RIGHT, I2_RIGHT, I3_LEFT, I4_LEFT], GPIO.OUT)
    GPIO.setup([LS_PIN, RS_PIN], GPIO.IN)

    # Default to "both wheels forward" until a motion call overrides it.
    set_wheel_direction(LEFT, forward=True)
    set_wheel_direction(RIGHT, forward=True)

    GPIO.add_event_detect(LS_PIN, GPIO.RISING, callback=_on_left_pulse)
    GPIO.add_event_detect(RS_PIN, GPIO.RISING, callback=_on_right_pulse)

    _state.initialised = True


def shutdown_rotate() -> None:
    """Stop both wheels and detach encoder interrupts.

    Safe to call repeatedly. Does not call ``GPIO.cleanup`` — the owner of
    the GPIO library (``main.py``) is responsible for that.
    """
    if not _state.initialised:
        return
    try:
        _state.pwm_left.ChangeDutyCycle(0)
        _state.pwm_right.ChangeDutyCycle(0)
    except Exception:
        pass
    try:
        GPIO.remove_event_detect(LS_PIN)
        GPIO.remove_event_detect(RS_PIN)
    except Exception:
        pass
    _state.initialised = False


# --- Direction helpers ------------------------------------------------------


def set_wheel_direction(side: int, forward: bool) -> None:
    """Set rotation direction of a single wheel via its H-bridge pins.

    Args:
        side: :data:`LEFT` or :data:`RIGHT`.
        forward: ``True`` to drive the wheel forward, ``False`` for reverse.

    Note:
        The "forward" polarity is the one used in ``main.py``:
        right wheel forward = (I1=HIGH, I2=LOW); left wheel forward =
        (I4=HIGH, I3=LOW). Reverse simply flips these.
    """
    if side == RIGHT:
        if forward:
            GPIO.output(I1_RIGHT, GPIO.HIGH)
            GPIO.output(I2_RIGHT, GPIO.LOW)
        else:
            GPIO.output(I1_RIGHT, GPIO.LOW)
            GPIO.output(I2_RIGHT, GPIO.HIGH)
    elif side == LEFT:
        if forward:
            GPIO.output(I4_LEFT, GPIO.HIGH)
            GPIO.output(I3_LEFT, GPIO.LOW)
        else:
            GPIO.output(I4_LEFT, GPIO.LOW)
            GPIO.output(I3_LEFT, GPIO.HIGH)
    else:
        raise ValueError(f"Unknown side: {side!r}")


def set_duty(side: int, duty: float) -> None:
    """Clamp and apply a duty cycle to one wheel's PWM channel.

    Args:
        side: :data:`LEFT` or :data:`RIGHT`.
        duty: requested duty cycle in [0, 100]. Values below
            :data:`MIN_START_DUTY` are passed through unclamped at the low
            end (so 0 means "stop"), and clamped at :data:`MAX_DUTY` on the
            high end.
    """
    if duty <= 0:
        duty = 0.0
    elif duty > MAX_DUTY:
        duty = MAX_DUTY
    if side == LEFT:
        _state.pwm_left.ChangeDutyCycle(duty)
    elif side == RIGHT:
        _state.pwm_right.ChangeDutyCycle(duty)
    else:
        raise ValueError(f"Unknown side: {side!r}")


def stop_all() -> None:
    """Cut PWM to both wheels (does not change direction pins)."""
    set_duty(LEFT, 0)
    set_duty(RIGHT, 0)


# --- Encoder access ---------------------------------------------------------


def reset_encoders() -> None:
    """Zero both wheel pulse counters atomically."""
    with _state.lock:
        _state.l_count = 0
        _state.r_count = 0


def get_pulse_counts() -> tuple[int, int]:
    """Return ``(left_pulses, right_pulses)`` accumulated since last reset."""
    with _state.lock:
        return _state.l_count, _state.r_count


def pulses_to_cm(pulses: int) -> float:
    """Convert a wheel-encoder pulse count to linear travel in cm."""
    return pulses / ENCODER_PULSES_PER_REV * WHEEL_CIRCUMFERENCE_CM


def cmps_to_duty_estimate(speed_cmps: float) -> float:
    """Open-loop guess for the duty cycle that yields a given wheel speed.

    Args:
        speed_cmps: desired wheel linear speed, cm/s.

    Returns:
        An initial duty cycle in [MIN_START_DUTY, MAX_DUTY]. The PID loop
        will refine this further at runtime.
    """
    if speed_cmps <= 0:
        return 0.0
    duty = speed_cmps / MAX_LINEAR_SPEED_CMPS * 100.0
    if duty < MIN_START_DUTY:
        duty = MIN_START_DUTY
    if duty > MAX_DUTY:
        duty = MAX_DUTY
    return duty


# --- Tiny incremental-PID, mirroring inc_pid.py -----------------------------


class WheelSpeedPID:
    """Incremental-form PID controller for a single wheel's linear speed.

    Matches the incremental update used in ``inc_pid.py``:
    ``du = Kp*(e-e1) + Ki*e + Kd*(e-2*e1+e2)``.
    """

    def __init__(self, target_cmps: float, init_duty: float,
                 gains: tuple[float, float, float] = DEFAULT_PID) -> None:
        """Initialise the controller.

        Args:
            target_cmps: target wheel linear speed in cm/s.
            init_duty: starting duty cycle (typically from
                :func:`cmps_to_duty_estimate`).
            gains: ``(Kp, Ki, Kd)`` tuple.
        """
        self._kp, self._ki, self._kd = gains
        self._target = target_cmps
        self._u = init_duty
        self._e = target_cmps
        self._e1 = target_cmps
        self._e2 = target_cmps

    def update(self, measured_cmps: float) -> float:
        """Feed a new measurement and get the next duty-cycle command.

        Args:
            measured_cmps: most recent measured wheel speed in cm/s.

        Returns:
            New duty-cycle command in [0, :data:`MAX_DUTY`].
        """
        self._e2, self._e1 = self._e1, self._e
        self._e = self._target - measured_cmps
        du = (self._kp * (self._e - self._e1)
              + self._ki * self._e
              + self._kd * (self._e - 2 * self._e1 + self._e2))
        self._u += du
        if self._u < 0:
            self._u = 0.0
        if self._u > MAX_DUTY:
            self._u = MAX_DUTY
        return self._u

    def retarget(self, target_cmps: float) -> None:
        """Update the speed setpoint without resetting accumulated state."""
        self._target = target_cmps


def require_initialised() -> None:
    """Raise if :func:`init_rotate` has not been called yet."""
    if not _state.initialised:
        raise RuntimeError(
            "motion.rotate is not initialised. Call init_rotate(pwm_left, "
            "pwm_right) once after configuring GPIO and PWM."
        )


# Convenience for callers that want to do quick speed sampling without
# spawning their own thread.
def sample_speeds(window_s: float) -> tuple[float, float]:
    """Block for ``window_s`` seconds and return ``(v_left, v_right)`` in cm/s.

    Note:
        This resets the encoder counters at the start of the window, so it
        should not be interleaved with other code that reads the same
        counters concurrently.
    """
    reset_encoders()
    time.sleep(window_s)
    l, r = get_pulse_counts()
    return pulses_to_cm(l) / window_s, pulses_to_cm(r) / window_s
