"""Dependency-free decisions for the RGB-D person-stop prototype."""
import math
from dataclasses import dataclass


def pair_is_fresh(stamps, received, now_ros, now_steady, timeout, sync_tolerance):
    """Both acquisition time and local arrival time must be recent."""
    values = (*stamps, *received, now_ros, now_steady, timeout, sync_tolerance)
    return (
        len(stamps) == len(received) == 2
        and all(math.isfinite(v) for v in values)
        and timeout > 0 and sync_tolerance >= 0
        and all(t > 0 and -0.05 <= now_ros-t <= timeout for t in stamps)
        and all(0 <= now_steady-t <= timeout for t in received)
        and abs(stamps[0]-stamps[1]) <= sync_tolerance
    )


def nearest_depth(samples, scale=1.0, minimum=0.05, maximum=10.0,
                  valid_fraction=0.5, minimum_samples=9):
    """Nearest valid optical depth in a person box; unknown returns None.

    Foreground objects in a box may cause an early stop. This is not a body
    surface reconstruction or a distance to the robot's physical perimeter.
    """
    if (not all(math.isfinite(v) for v in (scale, minimum, maximum, valid_fraction))
            or scale <= 0 or not 0 < minimum < maximum
            or not 0 < valid_fraction <= 1 or minimum_samples < 1):
        raise ValueError('invalid depth configuration')
    count = valid = 0
    nearest = math.inf
    for raw in samples:
        count += 1
        depth = float(raw)*scale
        if math.isfinite(depth) and minimum <= depth <= maximum:
            valid += 1
            nearest = min(nearest, depth)
    if valid < minimum_samples or valid < count*valid_fraction:
        return None
    return nearest


@dataclass(frozen=True)
class Decision:
    clear: bool
    reason: str
    nearest_m: object = None


class PersonStopPolicy:
    def __init__(self, stop_distance=1.0, release_distance=1.5):
        if (not all(math.isfinite(v) for v in (stop_distance, release_distance))
                or not 0 < stop_distance < release_distance):
            raise ValueError('require 0 < stop_distance < release_distance')
        self.stop_distance = stop_distance
        self.release_distance = release_distance
        self.blocked = True

    def fault(self, reason):
        self.blocked = True
        return Decision(False, reason)

    def evaluate(self, distances):
        if any(d is None or not math.isfinite(d) or d <= 0 for d in distances):
            return self.fault('person depth invalid')
        nearest = min(distances) if distances else None
        if nearest is not None and nearest <= self.stop_distance:
            self.blocked = True
        elif nearest is None or nearest >= self.release_distance:
            self.blocked = False
        return Decision(not self.blocked, 'person near' if self.blocked else 'clear', nearest)
