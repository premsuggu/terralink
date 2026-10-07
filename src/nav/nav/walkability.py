"""Turn `emap`'s published `traversability`/`is_valid` layers into a single
boolean "can a UGV drive here" mask - the one thing this whole package
replaces from `src/d3 (since removed; last present in commit 8a86dab)/my_bot`, which answered this question with a hardcoded
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


def compute_frontier_mask(
    walkable_mask: np.ndarray,
    is_valid: np.ndarray,
    min_unobserved_width_m: float = 0.0,
    resolution: float | None = None,
) -> np.ndarray:
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

        min_unobserved_width_m, resolution: if `min_unobserved_width_m > 0`
            (and `resolution` is given), frontier cells that belong to an
            unobserved region THINNER than this are dropped. Why: a strip of
            unobserved cells narrower than the robot cannot be a passage no
            matter what is inside it - and on a top-down map, thin unobserved
            strips are overwhelmingly the unscanned top of a thin WALL (a real
            0.15 m wall is only 1-2 cells wide), which looks exactly like
            "unobserved cells between two free areas". Left in, planners
            repeatedly "gamble" through walls (seen live, in every run).
            The width of a region is estimated from its largest inscribed
            circle; real unobserved space (occluded floor, the world beyond the
            scanned area) is thick and is kept. Default 0 = off.

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
    frontier = unobserved & adjacent_to_walkable
    if min_unobserved_width_m > 0 and resolution:
        frontier = _drop_thin_unobserved(frontier, unobserved, min_unobserved_width_m, resolution)
    return frontier


def _drop_thin_unobserved(
    frontier: np.ndarray, unobserved: np.ndarray, min_width_m: float, resolution: float
) -> np.ndarray:
    """Remove frontier cells whose LOCAL unobserved neighbourhood is too thin
    to hold a disc of diameter `min_width_m`.

    LOCAL matters: a thin wall-top seam usually runs into a big unobserved area
    (the world beyond the scanned region), so judging the whole connected region
    calls the seam thick. Instead, for each cell look at the deepest point of
    unobserved space within a robot-sized neighbourhood: a cell inside a strip
    `w` cells wide has distance ceil(w/2) to the nearest observed cell, so the
    largest disc near it is about 2*depth - 1 cells across (a slight
    under-estimate, which errs toward dropping marginal strips). Seam cells far
    from the open area see only shallow depth and are dropped; cells right at a
    real opening see the deep area and are kept.
    """
    from scipy import ndimage as ndi

    depth = ndi.distance_transform_edt(unobserved)  # distance (cells) to the nearest observed cell
    k = max(1, int(np.ceil(min_width_m / resolution / 2.0)))
    yy, xx = np.mgrid[-k : k + 1, -k : k + 1]
    footprint = (yy**2 + xx**2) <= k**2
    local_depth = ndi.maximum_filter(depth, footprint=footprint)
    width_cells = 2.0 * local_depth - 1.0
    return frontier & (width_cells * resolution >= min_width_m)
