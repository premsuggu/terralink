"""Time-hold ("latch") for a boolean mask that flickers.

The anomaly detector re-derives its mask from scratch on every map message,
and while the map is still being built its output is not stable: the cells
flagged at a tunnel mouth can vanish for a few messages and then reappear.
A planner that trusts each message literally sees the tunnel route appear and
disappear, and flips between it and whatever else is available - the UGV then
drives toward one, then the other.

`MaskLatch` keeps a cell "on" for `hold_sec` after the last message in which
it was on. It is deliberately dumb: no per-cell confidence, no decay - just a
timestamp per cell. Anything that must override it (a voxel-check verdict,
see `nav.resolved_regions`) is applied AFTER the latch, so a confirmed result
always wins immediately.

Pure NumPy, no ROS.
"""
from __future__ import annotations

import numpy as np


class MaskLatch:
    def __init__(self, hold_sec: float):
        self.hold_sec = float(hold_sec)
        self._last_on: np.ndarray | None = None

    def clear(self) -> None:
        self._last_on = None

    def update(self, mask: np.ndarray, now: float) -> np.ndarray:
        """Feed this message's mask at time `now` (seconds, any monotonic
        clock); returns the held mask. With `hold_sec <= 0` the input is
        returned unchanged (same object), so the feature costs nothing when
        off."""
        if self.hold_sec <= 0:
            return mask
        mask = np.asarray(mask, dtype=bool)
        if self._last_on is None or self._last_on.shape != mask.shape:
            self._last_on = np.full(mask.shape, -np.inf)
        self._last_on[mask] = now
        return (now - self._last_on) <= self.hold_sec
