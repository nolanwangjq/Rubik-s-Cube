"""Straight-line motion: drive forward or backward in a straight line."""

from __future__ import annotations

import time
from typing import Callable

from . import _hw
from ._hw import (
    LEFT,
    RIGHT,
    WheelSpeedPID,
)

FORWARD = "forward"
BACKWARD = "backward"

_PID_PERIOD_S = 0.1
_SPEED_WINDOW_S = 0.1


def straight(direction: str,
             mode: str,
             *,
             speed_cmps: float = 15.0,
             distance_cm: float | None = None,
             duration_s: float | None = None,
             stop_condition: Callable[[], bool] | None = None) -> None:
    """Drive the chassis in a straight line using closed-loop PID control.

    Args:
        direction: motion.straight.FORWARD or motion.straight.BACKWARD.
        mode: 
            "speed": runs continuously (capped by duration_s). 
            "distance": drives exactly distance_cm and then stops.
            "condition": runs continuously until stop_condition() returns True.
        speed_cmps: target linear speed, cm/s.
        distance_cm: required in "distance" mode, cm.
        duration_s: optional cap for "speed" mode.
        stop_condition: a callable returning a boolean, required for "condition" mode.
    """
    _hw.require_initialised()
    if direction not in (FORWARD, BACKWARD):
        raise ValueError(f"direction must be FORWARD or BACKWARD, got {direction!r}")
    if speed_cmps <= 0:
        raise ValueError("speed_cmps must be positive")

    is_forward = (direction == FORWARD)
    _hw.set_wheel_direction(LEFT, forward=is_forward)
    _hw.set_wheel_direction(RIGHT, forward=is_forward)

    init_duty = _hw.cmps_to_duty_estimate(speed_cmps)

    if mode == "speed":
        _straight_speed_mode(speed_cmps, init_duty, duration_s)
    elif mode == "distance":
        if distance_cm is None or distance_cm <= 0:
            raise ValueError("distance_cm must be positive in 'distance' mode")
        _straight_distance_mode(speed_cmps, init_duty, distance_cm)
    elif mode == "condition":
        if stop_condition is None:
            raise ValueError("stop_condition must be provided in 'condition' mode")
        _straight_condition_mode(speed_cmps, init_duty, stop_condition)
    else:
        raise ValueError(f"mode must be 'speed', 'distance', or 'condition', got {mode!r}")

    _hw.stop_all()


def _straight_condition_mode(target_v: float,
                             init_duty: float,
                             stop_condition: Callable[[], bool]) -> None:
    """Closed-loop straight motion until a custom condition evaluates to True."""
    pid_l = WheelSpeedPID(target_v, init_duty)
    pid_r = WheelSpeedPID(target_v, init_duty)

    _hw.set_duty(LEFT, init_duty)
    _hw.set_duty(RIGHT, init_duty)

    try:
        while True:
            # 优先检查视觉条件：如果满足，直接退出直线运动状态
            if stop_condition():
                return

            t_start = time.time()
            v_l, v_r = _hw.sample_speeds(_SPEED_WINDOW_S)
            
            _hw.set_duty(LEFT, pid_l.update(v_l))
            _hw.set_duty(RIGHT, pid_r.update(v_r))

            elapsed = time.time() - t_start
            time.sleep(max(0.0, _PID_PERIOD_S - elapsed))
    except KeyboardInterrupt:
        return

def _straight_speed_mode(target_v: float,
                         init_duty: float,
                         duration_s: float | None) -> None:
    """Closed-loop constant-speed straight motion."""
    # 这里使用的是 _hw.py 中统一的 PID 增益，
    # 如果左右轮电机差异大，可以像你原本代码中那样传入不同的 (Kp, Ki, Kd) 元组
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
            
            # sample_speeds 已经阻塞了 _SPEED_WINDOW_S，此处无需额外大量 sleep
            time.sleep(max(0.0, _PID_PERIOD_S - _SPEED_WINDOW_S))
    except KeyboardInterrupt:
        return

def _straight_distance_mode(target_v: float,
                            init_duty: float,
                            distance_cm: float) -> None:
    """Closed-loop straight motion by a fixed distance."""
    pid_l = WheelSpeedPID(target_v, init_duty)
    pid_r = WheelSpeedPID(target_v, init_duty)

    # Note: 累积距离需要依赖编码器，此处重置计数器
    _hw.reset_encoders()
    _hw.set_duty(LEFT, init_duty)
    _hw.set_duty(RIGHT, init_duty)

    # 累计行驶距离的变量（因为 _hw.sample_speeds 内部会重置编码器，我们需要在此处自行累加）
    total_dist_l = 0.0
    total_dist_r = 0.0