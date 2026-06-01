"""Ultrasonic ranging (KS103 sensor over I2C).

Typical usage:

    from perception.ultrasonic import start, get_distance, stop

    start()                  # spawns a background polling thread
    ...
    d_cm = get_distance()    # latest cached distance in cm; -1 if no read yet
    ...
    stop()                   # on shutdown
"""

from .ranger import (
    DEFAULT_I2C_ADDRESS,
    get_distance,
    read_distance_once,
    start,
    stop,
)

__all__ = [
    "DEFAULT_I2C_ADDRESS",
    "get_distance",
    "read_distance_once",
    "start",
    "stop",
]
