import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Calibration:
    pulses_per_ml: float = 555.555556
    extrusion_direction: int = -1
    maximum_step_rate_hz: float = 5000.0

    def __post_init__(self):
        if (
            not math.isfinite(self.pulses_per_ml)
            or self.pulses_per_ml <= 0
        ):
            raise ValueError(
                "pulses_per_ml must be finite and positive"
            )

        if self.extrusion_direction not in (-1, 1):
            raise ValueError(
                "extrusion_direction must be -1 or +1"
            )

        if (
            not math.isfinite(self.maximum_step_rate_hz)
            or self.maximum_step_rate_hz < 1
        ):
            raise ValueError(
                "maximum_step_rate_hz must be finite and >= 1"
            )

    def rate(self, value):
        if (
            not math.isfinite(value)
            or not 1 <= value <= self.maximum_step_rate_hz
        ):
            raise ValueError(
                f"Pulse rate must be 1.."
                f"{self.maximum_step_rate_hz:g} Hz"
            )

        rate = int(math.floor(value + 0.5))

        if rate > self.maximum_step_rate_hz:
            raise ValueError(
                "Rounded pulse rate exceeds configured maximum"
            )

        return rate

    def flow(self, ml_min):
        if not math.isfinite(ml_min) or ml_min <= 0:
            raise ValueError(
                "flow_ml_per_min must be finite and positive"
            )

        requested_rate = ml_min * self.pulses_per_ml / 60.0
        rate = self.rate(requested_rate)

        effective_flow = rate * 60.0 / self.pulses_per_ml

        return rate, effective_flow

    def volume(self, ml):
        if not math.isfinite(ml) or ml == 0:
            raise ValueError(
                "volume_ml must be finite and nonzero"
            )

        magnitude = abs(ml) * self.pulses_per_ml

        if (
            not math.isfinite(magnitude)
            or magnitude > 2147483647
        ):
            raise ValueError(
                "Volume exceeds signed 32-bit pulse range"
            )

        pulses = int(math.floor(magnitude + 0.5))

        if pulses < 1:
            raise ValueError(
                "Requested volume rounds to zero pulses"
            )

        volume_direction = 1 if ml > 0 else -1

        return (
            self.extrusion_direction
            * volume_direction
            * pulses
        )