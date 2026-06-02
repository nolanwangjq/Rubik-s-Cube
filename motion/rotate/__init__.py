"""Rotation primitives: in-place spin and circular orbit.

Typical usage:

    import RPi.GPIO as GPIO
    from motion.rotate import init_rotate, spin, orbit, CW, CCW, LEFT, RIGHT

    # ... existing GPIO + PWM setup in main.py ...
    init_rotate(pwm_left=pwmLeft, pwm_right=pwmRight)

    spin(direction=LEFT, mode="angle", angle_deg=90)
    orbit(direction=CCW, radius_cm=25, mode="speed", speed_cmps=8.0, duration_s=3.0)
"""

from .spin import spin
from .orbit import orbit
from ._hw import (
    init_rotate,
    shutdown_rotate,
    LEFT,
    RIGHT,
    CW,
    CCW,
)

__all__ = [
    "init_rotate",
    "shutdown_rotate",
    "spin",
    "orbit",
    "straight",   # <--- 新增直线运动
    "FORWARD",    # <--- 新增向前常量
    "BACKWARD",   # <--- 新增向后常量
    "LEFT",
    "RIGHT",
    "CW",
    "CCW",
]
