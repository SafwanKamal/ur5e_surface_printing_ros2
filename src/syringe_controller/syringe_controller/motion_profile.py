"""Timing model for the fixed distance-domain ramp in syringe_demo.ino."""

import math

START_RATE_HZ = 100.0
ACCEL_PULSES_S2 = 1000.0
MAX_PULSES = 3000


def move_duration(steps, rate):
    """Nominal seconds, including enable settling and the final HIGH pulse.

    Each interval uses the speed of the upcoming pulse. Loop/serial delays
    require the node's additional timeout margin. Keep constants synchronized
    with the firmware when changing the profile.
    """
    count = abs(steps)
    if not 1 <= count <= MAX_PULSES or not math.isfinite(rate) or not 1 <= rate <= 5000:
        raise ValueError("Move must use 1..3000 pulses at 1..5000 Hz")
    initial = min(START_RATE_HZ, rate)
    intervals_us = sum(
        math.ceil(1_000_000 / min(
            rate,
            math.sqrt(initial**2 + 2 * ACCEL_PULSES_S2 * min(i, count - 1 - i)),
        ))
        for i in range(1, count)
    )
    return (1000 + intervals_us + 10) / 1_000_000
