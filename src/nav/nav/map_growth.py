"""Is the map still being built?

While the UAV is still scanning, the map changes a lot from one message to the
next; once it has finished, it stops growing. A UGV that acts on SPECULATIVE
information (routes that only exist by assuming unobserved space is free,
exploration of the edge of the known map) while the map is still filling in is
acting on information that is about to be replaced - seen live, it set off on
gambles that a few seconds of further scanning would have shown to be pointless.

`MapGrowthTracker` answers the one question needed: over the last
`window_sec`, did the number of observed cells grow by more than
`min_growth_frac`? Measured on three recorded runs, that fraction was 3-30 % per
5 s while the UAV patrolled and exactly 0 % once it finished, so the signal is
clean.

Pure Python, no ROS and no NumPy.
"""
from __future__ import annotations

from collections import deque


class MapGrowthTracker:
    def __init__(self, window_sec: float = 10.0, min_growth_frac: float = 0.01):
        self.window_sec = window_sec
        self.min_growth_frac = min_growth_frac
        self._samples: deque[tuple[float, int]] = deque()

    def update(self, observed_cells: int, now: float) -> None:
        """Record the current count of observed cells at time `now`."""
        self._samples.append((now, int(observed_cells)))
        # keep one sample at or before the window start, drop anything older
        while len(self._samples) > 2 and self._samples[1][0] <= now - self.window_sec:
            self._samples.popleft()

    def growing(self, now: float) -> bool:
        """True while the map is still growing - and, deliberately, also
        before there is enough history to tell (at startup the map IS growing)."""
        if len(self._samples) < 2:
            return True
        t_old, c_old = self._samples[0]
        t_new, c_new = self._samples[-1]
        if t_new - t_old < self.window_sec * 0.5:
            return True  # not enough time covered yet to call it stable
        if now - t_new > self.window_sec:
            return False  # no map updates at all for a whole window: nothing is growing
        return (c_new - c_old) / max(c_old, 1) > self.min_growth_frac
