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


def compute_frontier_mask(walkable_mask: np.ndarray, is_valid: np.ndarray) -> np.ndarray:
    """The "frontier" concept from classical exploration literature
    (Yamauchi 1997): a cell that has never been observed, but sits directly
    next to a cell we already know is walkable. This is a NEW, separate
    concept from `compute_walkable_mask` above - it does not change what
    "walkable" means, it names a third category the existing binary split
    had no word for.

    Why this exists: `emap`'s UAV maps everything from directly overhead, so
    a structure that occludes the ground from above - a tunnel, culvert, or
    roofed passage - leaves every cell underneath permanently `is_valid=False`
    (the UAV's camera literally never gets a return from that ground), no
    matter how long or how thoroughly the area around it gets scanned. Under
    the existing walkable/non-walkable split, that's indistinguishable from a
    solid wall - `nav.prm_planner` can never route through it even when it's
    the ONLY way across (see docs/work-docs/nav/step05_frontier_tunnel_navigation.md
    for the full scenario this was built to address).

    A "frontier" cell is NOT claimed to be safe - it's explicitly the
    opposite: "unknown, but reachable, and worth physically investigating"
    (as opposed to unknown cells buried deep in never-approached space, which
    stay excluded exactly as before - only genuinely reachable unknowns
    become frontier). `nav.prm_planner.plan`'s `allow_frontier` option is
    what actually decides whether a planner is willing to tentatively route
    through a frontier cell; this function only IDENTIFIES which cells
    qualify, so a caller that never opts in to `allow_frontier` sees no
    behavior change at all from this function existing.

    Args:
        walkable_mask: (rows, cols) boolean array from `compute_walkable_mask`
            - i.e. cells already confirmed safe.
        is_valid: (rows, cols) boolean/0-1 array, decoded straight from the
            `/elevation_map` GridMap's `is_valid` layer (the SAME array
            `compute_walkable_mask` was given, not a different one - a cell
            that's unobserved in `is_valid` but happens to be True in
            `walkable_mask` from stale/mismatched inputs would be a caller
            bug, not something this function can detect).

    Returns:
        A new (rows, cols) boolean array - True where a cell is currently
        unobserved AND 4-connected-adjacent to at least one walkable cell.
        4-connectivity (not 8/diagonal) is the standard frontier-detection
        convention (Yamauchi's original and every descendant) - a diagonal
        neighbor doesn't imply the two cells are actually reachable from one
        another without also observing what's between them.
    """
    walkable_mask = np.asarray(walkable_mask, dtype=bool)
    unobserved = ~np.asarray(is_valid).astype(bool)

    # Pad with False on every side so a cell on the grid's own edge doesn't
    # need special-casing - a border cell simply has no walkable neighbor
    # off the edge of the map, which the padding already expresses correctly.
    padded = np.pad(walkable_mask, 1, mode="constant", constant_values=False)
    adjacent_to_walkable = (
        padded[:-2, 1:-1]  # neighbor one row up
        | padded[2:, 1:-1]  # neighbor one row down
        | padded[1:-1, :-2]  # neighbor one col left
        | padded[1:-1, 2:]  # neighbor one col right
    )
    return unobserved & adjacent_to_walkable
