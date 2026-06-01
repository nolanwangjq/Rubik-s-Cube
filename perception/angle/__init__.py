"""Heading / bearing angle perception (magnetic compass via ADC0832).

Typical usage:

    from perception.angle import start, get_angle, stop

    start()                # spawns a background polling thread
    ...
    heading_deg = get_angle()   # 0..360, 0 = north; -1.0 if no read yet
    ...
    stop()
"""

from .compass import (
    DEFAULT_ADC_CHANNEL,
    get_angle,
    get_raw,
    read_angle_once,
    start,
    stop,
)

__all__ = [
    "DEFAULT_ADC_CHANNEL",
    "get_angle",
    "get_raw",
    "read_angle_once",
    "start",
    "stop",
]
