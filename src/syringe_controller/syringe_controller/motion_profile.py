"""Timing model for the fixed distance-domain ramp in syringe_demo.ino."""

import math

START_RATE_HZ = 100.0
ACCEL_PULSES_S2 = 1000.0


def move_duration(steps, rate):
    """Nominal seconds, including enable settling and the final HIGH pulse.

    Each interval uses the speed of the upcoming pulse. Loop/serial delays
    require the node's additional timeout margin. Keep constants synchronized
    with the firmware when changing the profile.
    """
    count = abs(steps)
    if count != int(count) or not 1 <= count <= 2147483647 or not math.isfinite(rate) or not 1 <= rate <= 5000:
        raise ValueError("Move requires signed 32-bit nonzero pulses at 1..5000 Hz")
    initial = min(START_RATE_HZ, rate)
    # Sum only the ramp tails; long plateaus need no per-pulse iteration.
    tail = min((count - 1) // 2 + 1,
               math.ceil((rate**2 - initial**2) / (2 * ACCEL_PULSES_S2)))
    intervals_us = 0
    covered = 0
    for distance in range(tail):
        copies = 1 if 2 * distance == count - 1 else 2
        intervals_us += copies * math.ceil(1_000_000 / min(
            rate, math.sqrt(initial**2 + 2 * ACCEL_PULSES_S2 * distance)))
        covered += copies
    intervals_us += (count - covered) * math.ceil(1_000_000 / rate)
    intervals_us -= math.ceil(1_000_000 / initial)
    return (1000 + intervals_us + 10) / 1_000_000
