"""Check sensor-clock progress without comparing it to the host wall clock."""

import math


class SourceProgress:
    def __init__(self):
        self.stamp = None
        self.arrival = 0.0

    def observe(self, stamp, now):
        value = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if not math.isfinite(value) or value <= 0.0:
            return False
        if self.stamp is not None and value <= self.stamp:
            return False
        self.stamp, self.arrival = value, now
        return True

    def fresh(self, now, timeout):
        return self.stamp is not None and 0.0 <= now - self.arrival <= timeout

    def contains(self, stamp, timeout):
        value = float(stamp.sec) + float(stamp.nanosec) / 1e9
        return self.stamp is not None and -0.1 <= self.stamp - value <= timeout
