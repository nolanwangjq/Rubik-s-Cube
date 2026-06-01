"""Minimal ADC0832 driver (bit-banged on Pi GPIO).

Self-contained copy of the reference ``ADC0832.py`` so the rest of the
project doesn't have to put that file on ``sys.path``. Same pin defaults
and the same wiringpi-microsecond timing.

    CS  -> BCM 17
    CLK -> BCM 18
    DIO -> BCM 27
"""

from __future__ import annotations

import RPi.GPIO as GPIO
import wiringpi


ADC_CS = 17
ADC_CLK = 18
ADC_DIO = 27

_USDELAY = 2                  # half-period of CLK (max 400 kHz -> 2 us)
_T_CONVERT = 8 * 2 * _USDELAY  # conversion time before reading


def setup(cs: int = 17, clk: int = 18, dio: int = 27) -> None:
    """Configure GPIO pins for the ADC0832.

    Args:
        cs: BCM pin number wired to the ADC's CS line.
        clk: BCM pin number wired to the ADC's CLK line.
        dio: BCM pin number wired to the ADC's DIO line.

    Note:
        Assumes ``GPIO.setmode(GPIO.BCM)`` has been (or is about to be)
        called by the host program. Does not call ``GPIO.cleanup``.
    """
    global ADC_CS, ADC_CLK, ADC_DIO
    ADC_CS, ADC_CLK, ADC_DIO = cs, clk, dio
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ADC_CS, GPIO.OUT)
    GPIO.setup(ADC_CLK, GPIO.OUT)


def get_result(channel: int = 0) -> int:
    """Perform one ADC conversion on the chosen channel.

    Args:
        channel: ``0`` or ``1`` (ADC0832 has two single-ended channels).

    Returns:
        The 8-bit sample as an integer in [0, 255]. Returns ``0`` if the
        two read-back passes disagree (the chip's built-in consistency
        check failed).
    """
    GPIO.setup(ADC_DIO, GPIO.OUT)
    GPIO.output(ADC_CS, 0)

    GPIO.output(ADC_CLK, 0)
    GPIO.output(ADC_DIO, 1); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 1); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 0)
    GPIO.output(ADC_DIO, 1); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 1); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 0)
    GPIO.output(ADC_DIO, channel); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 1); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 0)
    GPIO.setup(ADC_DIO, GPIO.IN); wiringpi.delayMicroseconds(_T_CONVERT)

    dat1 = 0
    for _ in range(8):
        GPIO.output(ADC_CLK, 1); wiringpi.delayMicroseconds(_USDELAY)
        GPIO.output(ADC_CLK, 0); wiringpi.delayMicroseconds(_USDELAY)
        dat1 = (dat1 << 1) | GPIO.input(ADC_DIO)

    dat2 = 0
    for i in range(8):
        dat2 = dat2 | (GPIO.input(ADC_DIO) << i)
        GPIO.output(ADC_CLK, 1); wiringpi.delayMicroseconds(_USDELAY)
        GPIO.output(ADC_CLK, 0); wiringpi.delayMicroseconds(_USDELAY)

    GPIO.output(ADC_CLK, 1)
    GPIO.output(ADC_CS, 1); wiringpi.delayMicroseconds(_USDELAY)
    GPIO.output(ADC_CLK, 0); wiringpi.delayMicroseconds(_USDELAY)

    return dat1 if dat1 == dat2 else 0
