"""Combine the UAV's 2.5D map with the UGV's separate GROUND layer into the map the planner uses.

Why two maps: a 2.5D cell holds one height. Under a deck the UAV's cell holds the deck (flat, "walkable") while the UGV drives on the
floor beneath it; and cells under a roof are often never observed by the UAV at all. The UGV's depth camera therefore feeds a
separate ground map (emap, `ground_layer:=true`) that the node classifies with the same v2 rules and publishes as `ground_elevation`,
`ground_valid`, `ground_traversability`, `ground_confidence`. Offline (step11 doc, X-C4): on W4 the ground layer classified 902 cells the
UAV never saw, 100 % correctly.

The rule (one function, shared by the live planner node and the offline replay so they cannot drift apart): where the ground layer
has an observation, it WINS - the UGV looks at the floor from 0.5-4 m, the UAV from 4-7 m above the roof - and the cell counts as
observed. Everywhere else the UAV's layers are used unchanged. With no ground_* layers in the message this is the identity.
"""
from __future__ import annotations

import numpy as np

GROUND_KEYS = ("ground_elevation", "ground_valid", "ground_traversability")


def combine_ground(layers: dict) -> dict:
    """Return a layers dict whose `elevation`, `traversability` and `is_valid` include the ground layer (a new dict; inputs untouched)."""
    if not all(k in layers for k in GROUND_KEYS):
        return layers
    gv = np.asarray(layers["ground_valid"]) > 0.5
    out = dict(layers)
    out["traversability"] = np.where(gv, layers["ground_traversability"], layers["traversability"]).astype(layers["traversability"].dtype)
    out["elevation"] = np.where(gv, layers["ground_elevation"], layers["elevation"]).astype(layers["elevation"].dtype)
    out["is_valid"] = np.where(gv, 1.0, layers["is_valid"]).astype(layers["is_valid"].dtype)
    return out
