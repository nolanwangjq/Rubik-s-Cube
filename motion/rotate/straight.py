"""Straight-line motion module integrated with the motion package framework."""

from __future__ import annotations

import time
from typing import Callable
from . import _hw
from ._hw import LEFT, RIGHT

FORWARD = "forward"
BACKWARD = "backward"

class PositionalPID:
    """完全复制 pid_control.py 的位置式 PID 算法，以确保你调好的参数完全适用"""
    def __init__(self, P, I, D, target_speed):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.target_speed = target_speed
        self.err_pre = 0
        self.err_last = 0
        self.integral = 0

    def update(self, feedback_value):
        self.err_pre = self.target_speed - feedback_value
        self.integral += self.err_pre
        derivative = self.err_pre - self.err_last
        output = (self.Kp * self.err_pre) + (self.Ki * self.integral) + (self.Kd * derivative)
        self.err_last = self.err_pre
        
        # 限制 PWM 占空比在 0 到 100 之间
        return max(0.0, min(100.0, output))


def straight(direction: str,
             mode: str,
             *,
             target_speed_rps01: float = 1.9,
             stop_condition: Callable[[], bool] | None = None,
             gains_l: tuple[float, float, float] = (150.0, 0.06, 20.0),
             gains_r: tuple[float, float, float] = (150.0, 0.01, 23.0)) -> None:
    """使用 pid_control.py 的专属参数闭环走直线
    
    Args:
        direction: FORWARD (向前) 或 BACKWARD (向后)
        mode: 必须为 "condition"（由外部视觉条件触发停止）
        target_speed_rps01: 目标速度，单位为 圈/0.1秒 (默认 1.9)
        stop_condition: 外部传入的视觉判断函数（返回 True 时停车）
        gains_l: 左轮专属 PID 参数 (Kp, Ki, Kd)
        gains_r: 右轮专属 PID 参数 (Kp, Ki, Kd)
    """
    # 确保硬件层已初始化
    _hw.require_initialised()
    
    if direction not in (FORWARD, BACKWARD):
        raise ValueError(f"Invalid direction: {direction}")
    if mode != "condition":
        raise ValueError("This integration version strictly requires mode='condition' for state machine usage.")
    if stop_condition is None:
        raise ValueError("stop_condition callback must be provided.")

    # 1. 使用硬件抽象层统一控制车轮方向
    is_forward = (direction == FORWARD)
    _hw.set_wheel_direction(LEFT, forward=is_forward)
    _hw.set_wheel_direction(RIGHT, forward=is_forward)

    # 2. 初始化专属的位置式 PID 控制器
    pid_left = PositionalPID(*gains_l, target_speed_rps01)
    pid_right = PositionalPID(*gains_r, target_speed_rps01)

    # 3. 核心数学桥梁：通过底盘物理常数计算车轮周长
    # 585 个脉冲等于 1 圈，转换得到的 cm 数即为周长
    wheel_circumference = _hw.pulses_to_cm(585)

    # 给电机制动器一个初始占空比（对标原本的 origin_duty=5）
    _hw.set_duty(LEFT, 5.0)
    _hw.set_duty(RIGHT, 5.0)

    try:
        while True:
            # 状态机核心：每轮循环优先检查视觉算法是否让我们停止
            if stop_condition():
                break

            # 4. 采集当前真实的物理速度 (单位: cm/s)
            # 该函数内部会阻塞 0.1 秒进行采样，完美替代了原本的 time.sleep(0.1)
            v_l_cmps, v_r_cmps = _hw.sample_speeds(0.1)

            # 5. 单位转换：将 cm/s 转换为 圈/0.1秒 
            # 公式: (cm/s * 0.1s) / 周长 = 0.1秒内转过的圈数
            lspeed_rps01 = (v_l_cmps * 0.1) / wheel_circumference
            rspeed_rps01 = (v_r_cmps * 0.1) / wheel_circumference

            # 6. 运行 PID 计算出新的 PWM 占空比
            u_left = pid_left.update(lspeed_rps01)
            u_right = pid_right.update(rspeed_rps01)

            # 7. 应用到硬件
            _hw.set_duty(LEFT, u_left)
            _hw.set_duty(RIGHT, u_right)

    except KeyboardInterrupt:
        pass
    finally:
        # 退出该状态时，安全关闭所有电机
        _hw.stop_all()
