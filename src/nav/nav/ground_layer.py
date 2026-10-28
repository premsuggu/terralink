"""Combine the UAV's 2.5D map with the UGV's separate GROUND layer into the map the planner uses.

Why two maps: a 2.5D cell holds one height. Under a deck the UAV's cell holds the deck (flat, "walkable") while the UGV drives on the
floor beneath it; and cells under a roof are often never observed by the UAV at all. The UGV's depth camera therefore feeds a
separate ground map (emap, `ground_layer:=true`) that the node classifies with the same v2 rules and publishes as `ground_elevation`,
`ground_valid`, `ground_traversability`, `ground_confidence`. Offline (step11 doc, X-C4): on W4 the ground layer classified 902 cells the
UAV never saw, 100 % correctly.

The rule (one function, shared by the live planner node and the offline replay so they cannot drift apart): where the ground layer
has a confident observation (`MIN_CONFIDENCE`), it WINS - the UGV looks at the floor from 0.5-4 m, the UAV from 4-7 m above the roof - and the cell counts as
observed. Everywhere else the UAV's layers are used unchanged. With no ground_* layers in the message this is the identity.
"""
from __future__ import annotations

import numpy as np

GROUND_KEYS = ("ground_elevation", "ground_valid", "ground_traversability")

# A ground-layer cell overrides the UAV map only if its `ground_confidence` (coverage x 1/(1 + variance/0.1), emap.terrain_analysis) is at
# least this. Found in the first live freespace run (W4, 2026-10-28): a third of the camera's LETHAL cells (1235 of 3795) were FALSE -
# 1-3 cell specks on open floor, ~12 cm BELOW it (bad depth points), with median confidence 0.06 against 0.22 for real obstacles.
# Inflated by the robot radius they sealed the doors and the planner declared the goal walled off. On that snapshot 0.15 removed
# 78 % of the false LETHAL cells, kept 83 % of the ground cells, missed no more real hazards (309 vs 336 cells) and restored the
# route. A cell below the threshold simply falls back to the UAV map (and, under a roof, to the lidar floor map).
MIN_CONFIDENCE = 0.15


def combine_ground(layers: dict, min_confidence: float = MIN_CONFIDENCE) -> dict:
    """Return a layers dict whose `elevation`, `traversability` and `is_valid` include the ground layer (a new dict; inputs untouched)."""
    if not all(k in layers for k in GROUND_KEYS):
        return layers
    gv = np.asarray(layers["ground_valid"]) > 0.5
    if "ground_confidence" in layers:                          # (offline models without a confidence layer: every cell counts)
        gv &= np.asarray(layers["ground_confidence"]) >= min_confidence
    out = dict(layers)
    out["traversability"] = np.where(gv, layers["ground_traversability"], layers["traversability"]).astype(layers["traversability"].dtype)
    out["elevation"] = np.where(gv, layers["ground_elevation"], layers["elevation"]).astype(layers["elevation"].dtype)
    out["is_valid"] = np.where(gv, 1.0, layers["is_valid"]).astype(layers["is_valid"].dtype)
    return out
