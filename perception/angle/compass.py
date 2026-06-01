"""Magnetic-compass heading via ADC0832.

The compass module on this chassis outputs an analog voltage that maps
linearly to the 0..360 deg bearing range (0 deg = magnetic north). The
ADC0832 samples that voltage on channel 0 and returns an 8-bit code
``raw`` in [0, 255]; we convert::

    heading_deg = 360.0 * raw / 255.0

A background daemon thread polls the ADC continuously and caches the
latest heading so consumers can call :func:`get_angle` cheaply.
"""

from __future__ import annotations

import threading
import time

from . import _adc0832


DEFAULT_ADC_CHANNEL = 0
_DEFAULT_POLL_INTERVAL_S = 0.05

# Sentinel returned by ``get_angle``/``get_raw`` before the first sample.
_NO_READ = -1.0


class _CompassState:
    """Holds the polling thread and the latest sample."""

    def __init__(self) -> None:
        self.angle_deg: float = _NO_READ
        self.raw: int = -1
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.setup_done = False


_state = _CompassState()


def _raw_to_degrees(raw: int) -> float:
    """Map an 8-bit ADC code to a bearing in degrees.

    Args:
        raw: ADC sample, expected to be in [0, 255].

    Returns:
        Bearing in degrees, in [0, 360).
    """
    deg = 360.0 * float(raw) / 255.0
    if deg >= 360.0:
        deg -= 360.0
    return deg


def _poll_loop(channel: int, poll_interval_s: float) -> None:
    """Background loop: read ADC -> cache, until stopped.

    Args:
        channel: ADC0832 channel (0 or 1).
        poll_interval_s: sleep between consecutive samples.
    """
    while not _state.stop_event.is_set():
        try:
            raw = _adc0832.get_result(channel)
            deg = _raw_to_degrees(raw)
            with _state.lock:
                _state.raw = raw
                _state.angle_deg = deg
        except Exception:
            time.sleep(0.05)
        if _state.stop_event.wait(poll_interval_s):
            break


def start(channel: int = DEFAULT_ADC_CHANNEL,
          poll_interval_s: float = _DEFAULT_POLL_INTERVAL_S,
          cs_pin: int = 17,
          clk_pin: int = 18,
          dio_pin: int = 27) -> None:
    """Configure the ADC and spawn the background polling thread.

    Safe to call multiple times; subsequent calls are no-ops while a
    polling thread is already running.

    Args:
        channel: ADC0832 channel to read (0 or 1).
        poll_interval_s: target spacing between samples.
        cs_pin: BCM pin for ADC CS (default 17).
        clk_pin: BCM pin for ADC CLK (default 18).
        dio_pin: BCM pin for ADC DIO (default 27).
    """
    if _state.thread is not None and _state.thread.is_alive():
        return
    if not _state.setup_done:
        _adc0832.setup(cs=cs_pin, clk=clk_pin, dio=dio_pin)
        _state.setup_done = True
    _state.stop_event.clear()
    _state.thread = threading.Thread(
        target=_poll_loop,
        args=(channel, poll_interval_s),
        daemon=True,
        name="compass-angle",
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


def get_angle() -> float:
    """Return the most recent bearing in degrees.

    Returns:
        Bearing in [0, 360), or ``-1.0`` if no successful sample has been
        captured yet.
    """
    with _state.lock:
        return _state.angle_deg


def get_raw() -> int:
    """Return the most recent raw 8-bit ADC code, or ``-1`` if none yet."""
    with _state.lock:
        return _state.raw


def read_angle_once(channel: int = DEFAULT_ADC_CHANNEL) -> float:
    """Blocking single-shot read, independent of the background poller.

    Args:
        channel: ADC0832 channel (0 or 1).

    Returns:
        Bearing in degrees, in [0, 360).
    """
    if not _state.setup_done:
        _adc0832.setup()
        _state.setup_done = True
    raw = _adc0832.get_result(channel)
    return _raw_to_degrees(raw)
