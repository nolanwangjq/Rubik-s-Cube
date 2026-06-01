"""KS103 ultrasonic rangefinder driver (I2C, wiringpi backend).

The KS103 protocol used here matches the reference example
``i2c_range_wiringpi.py``:

* write ``0xb0`` to register ``0x02`` to trigger a 0..5 m ranging,
* wait at least 33 ms,
* read registers ``0x02`` (high byte) and ``0x03`` (low byte); the
  16-bit result is the distance in millimetres.

A background daemon thread polls the sensor continuously and caches the
most recent reading so consumers can call :func:`get_distance` without
blocking.
"""

from __future__ import annotations

import threading
import time

import wiringpi as wpi


DEFAULT_I2C_ADDRESS = 0x74
_WRITE_CMD = 0xb0          # 0..5 m range, result in millimetres
_TRIGGER_REG = 0x02
_RESULT_HIGH_REG = 0x02
_RESULT_LOW_REG = 0x03
_MIN_DELAY_MS = 33         # KS103 datasheet minimum between trigger and read
_DEFAULT_POLL_INTERVAL_S = 0.05

# Sentinel returned by ``get_distance`` before the first successful read.
_NO_READ = -1.0


class _RangerState:
    """Holds the polling thread and the latest reading."""

    def __init__(self) -> None:
        self.distance_cm: float = _NO_READ
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.handle: int | None = None


_state = _RangerState()


def _poll_loop(address: int, poll_interval_s: float) -> None:
    """Background loop: trigger -> wait -> read -> cache, until stopped.

    Args:
        address: I2C address of the KS103.
        poll_interval_s: extra sleep between successive reads (on top of
            the mandatory 33 ms trigger-to-read delay).
    """
    h = wpi.wiringPiI2CSetup(address)
    _state.handle = h
    delay_ms = max(_MIN_DELAY_MS, int(poll_interval_s * 1000))

    while not _state.stop_event.is_set():
        try:
            wpi.wiringPiI2CWriteReg8(h, _TRIGGER_REG, _WRITE_CMD)
            wpi.delay(delay_ms)
            hi = wpi.wiringPiI2CReadReg8(h, _RESULT_HIGH_REG)
            lo = wpi.wiringPiI2CReadReg8(h, _RESULT_LOW_REG)
            dist_mm = (hi << 8) + lo
            dist_cm = dist_mm / 10.0
            with _state.lock:
                _state.distance_cm = dist_cm
        except Exception:
            # Transient I2C errors should not kill the polling loop; just
            # leave the cached value stale and retry next iteration.
            time.sleep(0.05)


def start(address: int = DEFAULT_I2C_ADDRESS,
          poll_interval_s: float = _DEFAULT_POLL_INTERVAL_S) -> None:
    """Spawn the background polling thread.

    Safe to call multiple times; subsequent calls are no-ops while a
    polling thread is already running.

    Args:
        address: I2C address of the KS103 (default ``0x74``).
        poll_interval_s: target spacing between consecutive sensor reads.
            Clamped to the KS103 minimum of 33 ms.
    """
    if _state.thread is not None and _state.thread.is_alive():
        return
    _state.stop_event.clear()
    _state.thread = threading.Thread(
        target=_poll_loop,
        args=(address, poll_interval_s),
        daemon=True,
        name="ultrasonic-ranger",
    )
    _state.thread.start()


def stop(timeout_s: float = 1.0) -> None:
    """Signal the polling thread to exit and wait for it to finish.

    Args:
        timeout_s: maximum time to wait for the thread to join.
    """
    _state.stop_event.set()
    if _state.thread is not None:
        _state.thread.join(timeout=timeout_s)
        _state.thread = None


def get_distance() -> float:
    """Return the most recent distance reading in centimetres.

    Returns:
        Distance in cm, or ``-1.0`` if no successful read has happened
        yet (e.g. :func:`start` was never called, or the sensor has not
        produced a value yet).
    """
    with _state.lock:
        return _state.distance_cm


def read_distance_once(address: int = DEFAULT_I2C_ADDRESS) -> float:
    """Blocking single-shot read, independent of the background poller.

    Useful for diagnostics or one-off measurements where running a
    polling thread would be overkill.

    Args:
        address: I2C address of the KS103.

    Returns:
        Distance in cm.
    """
    h = wpi.wiringPiI2CSetup(address)
    wpi.wiringPiI2CWriteReg8(h, _TRIGGER_REG, _WRITE_CMD)
    wpi.delay(_MIN_DELAY_MS)
    hi = wpi.wiringPiI2CReadReg8(h, _RESULT_HIGH_REG)
    lo = wpi.wiringPiI2CReadReg8(h, _RESULT_LOW_REG)
    return ((hi << 8) + lo) / 10.0
