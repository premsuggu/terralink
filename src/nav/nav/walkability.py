"""Turn `emap`'s published `traversability`/`is_valid` layers into a single
boolean "can a UGV drive here" mask - the one thing this whole package
replaces from `src/d3/my_bot`, which answered this question with a hardcoded
BGR color-range threshold on a live top-down camera image
(`waypoints_server.cpp::process_image`: `cv::inRange(cv_image, (100,100,100),
(180,180,180))`), tuned to one specific world's floor-gray rendering.

Why NOT an elevation threshold (e.g. "flat/near-zero elevation = walkable"):
that was seriously considered and rejected. `emap`'s own construction-site
world already proved absolute elevation is not a safe proxy for "flat" -
that world's flat ground sits at world z=+1.5, not 0 (see
docs/work-docs/emap/step12_construction_site_world.md, Section 3) - so a
rule like "walkable if abs(elevation) < eps" would have called that entire
site one giant obstacle, and would need hand-tuning per world regardless.

`emap.traversability.compute_traversability` (step 7) already solves this
correctly and generally: it scores each cell from slope, step-height, and
roughness *relative to its neighbors*, not from an absolute height. That's
exactly the baseline-independent property navigation needs, and it's already
built, tested, and running in the live `elevation_mapping_node` - so this
module does nothing more than apply it.

`is_valid` is checked too, and separately: a cell `compute_traversability`
has never actually scored (the UAV hasn't flown over it yet) is NOT the same
as a cell confirmed flat. `traversability.py`'s own docstring is explicit
that its `EASY` default for unobserved cells is a fail-safe for *other*
callers (so an unmasked cell doesn't accidentally read LETHAL), not a claim
that unscanned ground is safe to drive across - so a UGV planner must check
`is_valid` itself rather than trust that default. See the discussion this
design followed in docs/work-docs/nav/00_concepts.md.
"""
from __future__ import annotations

import numpy as np

from emap.traversability import LETHAL


def compute_walkable_mask(traversability: np.ndarray, is_valid: np.ndarray) -> np.ndarray:
    """A cell is walkable only if it's both been observed AND scored better
    than LETHAL by `compute_traversability` (i.e. EASY or DIFFICULT).

    Args:
        traversability: (rows, cols) array of {LETHAL, DIFFICULT, EASY}
            scores, decoded straight from the `/elevation_map` GridMap's
            `traversability` layer (see `emap.utils.gridmap_utils.decode_gridmap`).
        is_valid: (rows, cols) boolean/0-1 array, decoded from the same
            message's `is_valid` layer.

    Returns:
        A new (rows, cols) boolean array - True where a UGV may be routed.
    """
    return np.asarray(is_valid).astype(bool) & (np.asarray(traversability) != LETHAL)
