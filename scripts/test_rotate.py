#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manual smoke-test for ``motion.rotate``.

Run on the robot with::

    python3 scripts/test_rotate.py

Each test is gated by an ``input()`` prompt so you can inspect the chassis
between runs (and abort with Ctrl-C). All tests reuse the same GPIO / PWM
setup that ``main.py`` performs at boot.
"""

import os
import sys
import time

# Make the project root importable when this script is launched directly.
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import RPi.GPIO as GPIO

from motion.rotate import (
    CCW,
    CW,
    LEFT,
    RIGHT,
    init_rotate,
    orbit,
    shutdown_rotate,
    spin,
)


# --- GPIO / PWM bring-up (mirrors main.py) ----------------------------------
EA, I2, I1, EB, I4, I3 = (13, 19, 26, 16, 20, 21)
FREQUENCY = 55

GPIO.setmode(GPIO.BCM)
GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
GPIO.output([EA, I2, EB, I3], GPIO.LOW)
GPIO.output([I1, I4], GPIO.HIGH)

pwmRight = GPIO.PWM(EA, FREQUENCY)
pwmLeft = GPIO.PWM(EB, FREQUENCY)
pwmRight.start(0)
pwmLeft.start(0)

init_rotate(pwm_left=pwmLeft, pwm_right=pwmRight)


def step(title):
    """Pause until the user confirms the next sub-test should run."""
    print("\n=== %s ===" % title)
    input("press <enter> to start, Ctrl-C to abort ... ")


try:
    # 1. Spin in place, speed mode: keep going for 2 s at 90 deg/s CCW.
    step("spin / speed mode / LEFT (CCW) / 90 dps / 2 s")
    spin(direction=LEFT, mode="speed", angular_speed_dps=90.0, duration_s=2.0)
    time.sleep(0.5)

    # 2. Spin in place, angle mode: rotate exactly 90 degrees to the right.
    step("spin / angle mode / RIGHT / 90 deg")
    spin(direction=RIGHT, mode="angle", angular_speed_dps=60.0, angle_deg=90.0)
    time.sleep(0.5)

    # 3. Spin in place, angle mode: 180 deg to the left.
    step("spin / angle mode / LEFT / 180 deg")
    spin(direction=LEFT, mode="angle", angular_speed_dps=60.0, angle_deg=180.0)
    time.sleep(0.5)

    # 4. Orbit, speed mode: r = 25 cm, CCW, 8 cm/s, 3 s.
    step("orbit / speed mode / CCW / r=25 cm / 8 cm/s / 3 s")
    orbit(direction=CCW, radius_cm=25.0, mode="speed",
          speed_cmps=8.0, duration_s=3.0)
    time.sleep(0.5)

    # 5. Orbit, angle mode: r = 25 cm, CW, sweep 90 deg around the centre.
    step("orbit / angle mode / CW / r=25 cm / 90 deg sweep")
    orbit(direction=CW, radius_cm=25.0, mode="angle",
          speed_cmps=8.0, angle_deg=90.0)
    time.sleep(0.5)

    # 6. Orbit, angle mode: tighter circle, full lap CCW.
    step("orbit / angle mode / CCW / r=15 cm / 360 deg sweep")
    orbit(direction=CCW, radius_cm=15.0, mode="angle",
          speed_cmps=6.0, angle_deg=360.0)

    print("\nAll tests finished.")

except KeyboardInterrupt:
    print("\nAborted by user.")
finally:
    shutdown_rotate()
    pwmRight.stop()
    pwmLeft.stop()
    GPIO.cleanup()
