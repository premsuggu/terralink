"""Detect where the 2.5D `/elevation_map` is probably LYING, not just where
it's LETHAL - the missing piece step05 found live and documented as its own
honest limitation (see docs/work-docs/nav/step05_frontier_tunnel_navigation.md,
"The honest finding").

The problem this solves: a depth camera looking straight down at a solid,
opaque overhang (a tunnel roof, a low bridge deck, anything blocking a
straight-down view of the real ground) does NOT produce `is_valid=False`
underneath it - it correctly measures the *overhang's own height*, and
`compute_traversability` correctly flags a LETHAL step where that height
meets the real floor's height nearby. Every layer involved is functioning
exactly as designed; the elevation map is simply incapable of recording that
two different surfaces exist at the same (x, y). `compute_frontier_mask`
(step05) is built entirely around `is_valid=False` and, correctly, does NOT
fire here - there is no missing data, just wrong-but-confident data.

REAL BUG FOUND LIVE, TWICE, at two different design stages (this version is
the second fix):

1. The ORIGINAL implementation looked for this shape using only LOCAL,
   single-scan-line geometry (a thin LETHAL band, walkable on both sides, at
   a measurably different elevation than its surroundings). It worked on
   synthetic test cases but, live, flagged points that looked essentially
   random to a human watching the sim.
2. The FIRST REDESIGN (connected-component + KD-tree nearest-gap, no
   elevation check at all) fixed the "random points" problem - flagged
   cells were now tightly concentrated at real, close-but-disconnected
   regions of the map, not scattered arbitrarily - but introduced a
   DIFFERENT, more serious bug, found during this redesign's own live
   verification (an advisor review caught it before it shipped as
   "working"): two genuinely SEPARATE ROOMS, connected by nothing but a
   real, ordinary, uniformly-thin wall (this project's walls are 0.15m =
   1-2 cells at 0.1m resolution - well within any reasonable thickness
   cap), are STRUCTURALLY INDISTINGUISHABLE from a real occluded tunnel
   under pure topology-plus-thickness reasoning: both are "two large,
   close, disconnected components with a thin all-LETHAL band between
   them." A synthetic reproduction (two 13+ m² rooms, a uniform 2-cell
   gap, no island anywhere) flagged 8 cells on a completely genuine wall.
   This is not a corner case - it's the single most common wall geometry
   in every one of this project's test worlds.

This version fixes bug 2 by bringing back what bug 1's fix threw away - an
elevation-based discriminator - but applies it to whole COMPONENTS instead
of individual scan lines, keeping bug 1's actual fix (global topology,
robust to local noise) while restoring bug-1-fix's missing piece (only a
genuine, differently-elevated ISLAND sitting BETWEEN two larger regions is
evidence of an occluded passage; a direct gap between two same-height
regions with nothing between them is just... a wall, indistinguishable from
any other wall, and must not be flagged). See
docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md's "Anomaly detection
v2" section for the full live evidence trail for both bugs and both fixes.

The algorithm, in one sentence: label connected components of walkable
space, split them into "rooms" (large, real regions) and "islands"
(small-to-medium components too big to be sensor/robot noise but too small
to be a real room - exactly what a tunnel/culvert interior's own roof
reading looks like); for every island with at least two ROOM neighbors that
are each close, thinly-connected, and at a measurably different elevation
than the island itself, flag the connecting LETHAL bands on both sides.
Two rooms are NEVER compared directly to each other - only through a
qualifying island in between - which is exactly what structurally rules out
the room-to-room-across-a-real-wall false positive above: a real wall has
no island on it at all.

"Anomalous" is NOT a claim that a cell is safe. It means "the 2.5D data here
is suspicious enough that it should be checked against ground truth (the
bounded 3D voxel map, `nav.voxel_map`, built in step06 Phase 1) before either
trusting or rejecting it" - the trigger for Phase 3's planner integration,
not a walkability verdict on its own. This module only identifies candidate
cells; nothing here changes planning behavior by itself (mirrors
`compute_frontier_mask`'s own "identify, don't decide" split from step05).
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

# 8-connectivity for component labeling: two walkable cells that only touch
# diagonally are still treated as the same reachable region. This matters
# here specifically - using 4-connectivity would fragment a single real room
# into multiple "components" at diagonal pinch points (e.g. a doorway
# entered at an angle), which would then spuriously look like two close,
# disconnected regions and defeat the whole point of this algorithm.
_EIGHT_CONNECTED = np.ones((3, 3), dtype=int)


def compute_anomaly_mask(
    elevation: np.ndarray,
    walkable_mask: np.ndarray,
    is_valid: np.ndarray,
    resolution: float,
    max_gap_distance_m: float = 1.5,
    max_lethal_band_width_m: float = 0.6,
    min_room_area_m2: float = 3.0,
    min_island_area_m2: float = 0.15,
    min_elevation_diff_m: float = 0.15,
) -> np.ndarray:
    """Flag LETHAL cells that sit between a real ROOM and a genuine,
    differently-elevated ISLAND component sandwiched between two such rooms
    - the signature of a real occluded passage (tunnel, culvert), not a
    direct room-to-room gap across an ordinary wall (see this module's own
    top docstring for the live-found bug this specifically rules out).

    Why an island is required, not just "two close disconnected regions":

    A 2.5D elevation map cannot, by construction, tell "unmapped passage
    under an overhang" apart from "genuine solid wall" using thickness and
    distance alone - both produce an identical thin, all-LETHAL band between
    two disconnected regions of walkable space. The one structural
    difference a REAL occluded passage has that a plain wall never does: if
    the overhang is wide enough to have its own flat interior (true of any
    tunnel/culvert a UGV could plausibly use), the camera reads that
    interior as its own genuinely walkable "island" - at the WRONG
    (overhang's) elevation, sandwiched between the two rooms it connects.
    An ordinary wall has no such island - there is nothing at all in
    between the two rooms it separates, just more of the same wall. Looking
    for the island (not just the gap) is what makes this distinguishable.

    Size ranges are what separate the three roles (rooms, islands, and
    noise/artifacts too small to matter) - see `min_room_area_m2` and
    `min_island_area_m2` below for the real, live-measured numbers these
    were calibrated against (a parked robot's own chassis, ~0.09 m²; this
    project's real tunnel interior, ~0.37 m²; this project's real rooms,
    ~28 m² - three size classes separated by roughly an order of magnitude
    each, leaving generous margin either side of every threshold).

    Args:
        elevation: (rows, cols) elevation layer, decoded straight from
            `/elevation_map`.
        walkable_mask: (rows, cols) boolean array from
            `nav.walkability.compute_walkable_mask`.
        is_valid: (rows, cols) boolean/0-1 array from the same message.
        resolution: meters per cell - converts the distance/area parameters
            into cell counts.
        max_gap_distance_m: only island-to-room pairs whose nearest points
            are within this real-world distance are even considered
            (default 1.5m).
        max_lethal_band_width_m: the connecting line between an island and a
            candidate room neighbor must be no thicker than this to count as
            a thin step-edge artifact rather than a genuinely thick, solid
            obstacle (default 0.6m, carried over from earlier live-tuned
            values - docs/work-docs/nav/step06_hybrid_3d_voxel_navigation.md,
            Phase 2).
        min_room_area_m2: a component must be at least this large to serve
            as one of the two (or more) "sides" of a candidate gap (default
            3.0 m² - comfortably above any plausible tunnel interior,
            comfortably below this project's real rooms, ~28 m², leaving
            wide margin on both sides). Components below this size are
            never treated as a room - only as a possible island, or as
            nothing at all (see min_island_area_m2).
        min_island_area_m2: a component must be at least this large to serve
            as the candidate "island" between two rooms (default 0.15 m² -
            safely above a parked robot's own chassis, ~0.09 m², the exact
            live-found false positive `min_component_area_m2` fixed in the
            first version of this redesign; safely below min_room_area_m2,
            so a real room is never mistaken for a candidate island either).
        min_elevation_diff_m: the island's own mean elevation must differ
            from a candidate room neighbor's mean elevation by at least this
            much for that connection to count (default 0.15m, carried over
            from the original per-scan-line heuristic's own threshold -
            solves the identical problem here: rejecting an ordinary,
            single-elevation floor that happens to read as two components
            for some unrelated reason, e.g. a genuinely unresolved gap that
            hasn't been checked by frontier logic yet).

    Returns:
        A new (rows, cols) boolean array - True on the LETHAL cells that lie
        on a flagged island-to-room connecting band (plus their immediate
        LETHAL neighbors, so the flagged region has enough width for a
        planner to actually route a line through it). Cells belonging to any
        walkable component - room or island - are never flagged themselves;
        only the LETHAL cells a planner would otherwise refuse to cross are
        marked tentative.
    """
    elevation = np.asarray(elevation, dtype=np.float32)
    walkable_mask = np.asarray(walkable_mask, dtype=bool)
    is_valid = np.asarray(is_valid).astype(bool)
    rows, cols = walkable_mask.shape

    # Same LETHAL definition every version of this module has used: actually
    # observed AND actually scored non-walkable. An unobserved cell
    # (is_valid=False) is a different concept (frontier, step05) - a
    # candidate connecting line that passes through unobserved territory
    # isn't confirmed to be a data-artifact gap at all, so it's rejected
    # below rather than guessed at.
    lethal_mask = is_valid & ~walkable_mask

    anomaly = np.zeros((rows, cols), dtype=bool)

    labeled, num_components = ndimage.label(walkable_mask, structure=_EIGHT_CONNECTED)
    if num_components < 2:
        # Nothing to compare a single (or zero) component against.
        return anomaly

    max_gap_cells = max_gap_distance_m / resolution
    max_band_cells = max(1, int(round(max_lethal_band_width_m / resolution)))
    min_room_cells = min_room_area_m2 / (resolution ** 2)
    min_island_cells = min_island_area_m2 / (resolution ** 2)

    component_sizes = ndimage.sum(
        np.ones_like(walkable_mask, dtype=int), labeled, index=np.arange(1, num_components + 1)
    )
    # Two DISJOINT roles by construction - a component is either large enough
    # to be a room, or in the (much lower) island range, never both. This is
    # exactly what structurally prevents two real rooms from ever being
    # compared to each other directly: a room is never eligible to play the
    # island role in the loop below, so the room-to-room-across-a-wall
    # pairing this version fixes simply cannot be constructed.
    room_labels = {
        label for label, size in zip(range(1, num_components + 1), component_sizes)
        if size >= min_room_cells
    }
    island_labels = {
        label for label, size in zip(range(1, num_components + 1), component_sizes)
        if min_island_cells <= size < min_room_cells
    }
    if not room_labels or not island_labels:
        # Nothing that could possibly form a valid room-island-room chain -
        # covers both "every component is a real room, no islands at all"
        # (matches the live-found room-to-room-wall bug case exactly - two
        # rooms, nothing else, now correctly producing zero flags) and
        # "only small artifacts exist, no real rooms yet" (early in a
        # patrol, before enough of the map has been scanned).
        return anomaly

    relevant_labels = room_labels | island_labels
    boundary_mask = _walkable_boundary(walkable_mask)
    component_boundary_coords: dict[int, np.ndarray] = {}
    component_mean_elev: dict[int, float] = {}
    for label_id in relevant_labels:
        component_cells = labeled == label_id
        coords = np.argwhere(component_cells & boundary_mask)
        if coords.size == 0:
            # A component with no 4-connected non-walkable neighbor at all -
            # only possible if it fills the entire map with no edge cell
            # bordering anything else, an edge case worth handling rather
            # than crashing on, even though it won't occur in practice.
            coords = np.argwhere(component_cells)
        component_boundary_coords[label_id] = coords
        # Whole-component mean, not just the boundary - a simple, robust
        # elevation estimate for what is usually a fairly flat real
        # surface (a room's floor, or a tunnel roof's own flat interior),
        # rather than relying on a handful of possibly-noisy edge cells.
        component_mean_elev[label_id] = float(elevation[component_cells].mean())

    trees = {label_id: cKDTree(coords) for label_id, coords in component_boundary_coords.items()}

    for island in sorted(island_labels):
        island_elev = component_mean_elev[island]
        qualifying_connections: list[list[tuple[int, int]]] = []

        for room in sorted(room_labels):
            # Nearest point in `room`'s boundary to every boundary point of
            # `island`, then the overall minimum - the same KD-tree
            # "shortest distance between two point sets" technique as
            # before, just applied to an island/room pair instead of a
            # room/room pair.
            dists, nearest_room_idx = trees[room].query(component_boundary_coords[island])
            k = int(np.argmin(dists))
            if dists[k] > max_gap_cells:
                continue

            point_island = tuple(component_boundary_coords[island][k])
            point_room = tuple(component_boundary_coords[room][nearest_room_idx[k]])

            interior = _line_cells(point_island, point_room)[1:-1]
            if not interior:
                continue  # already adjacent - shouldn't happen given 8-connected labeling
            if not all(lethal_mask[r, c] for r, c in interior):
                # Not confirmed LETHAL the whole way (either genuinely
                # walkable, meaning these weren't really disconnected, or
                # unobserved territory - frontier's concern, not this one's).
                continue
            if len(interior) > max_band_cells:
                continue  # too thick to be a step-edge artifact - a real wall
            if abs(island_elev - component_mean_elev[room]) < min_elevation_diff_m:
                # THE discriminator this version restores: no measurable
                # elevation difference means this could just as easily be an
                # ordinary same-height floor that happens to read as two
                # components for some other reason - not evidence of an
                # occluded overhang.
                continue

            qualifying_connections.append(interior)

        if len(qualifying_connections) < 2:
            # A real occluded passage connects to real ground on (at least)
            # two sides - an island with only one qualifying room neighbor
            # (a dead end, or simply not enough of the map scanned yet to
            # tell) isn't enough evidence on its own.
            continue

        for interior in qualifying_connections:
            for r, c in interior:
                anomaly[r, c] = True
            # Thicken the flagged band by one cell in every direction
            # (restricted to genuinely LETHAL cells only) - the connecting
            # line itself can be a single diagonal thread of cells, too
            # narrow for a PRM edge or the UGV's own footprint to reliably
            # land on.
            for r, c in list(interior):
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < rows and 0 <= cc < cols and lethal_mask[rr, cc]:
                            anomaly[rr, cc] = True

    return anomaly


def _walkable_boundary(walkable_mask: np.ndarray) -> np.ndarray:
    """A walkable cell counts as a boundary cell if at least one of its
    4-connected neighbors (or the map edge itself) is NOT walkable. Uses
    4-connectivity here deliberately, distinct from the 8-connectivity used
    for component labeling above - this only needs to find cells genuinely
    adjacent to a break in the region, not reason about diagonal reachability.
    """
    rows, cols = walkable_mask.shape
    boundary = np.zeros((rows, cols), dtype=bool)
    # Compare each cell against its 4 neighbors; a cell with any neighbor
    # off the edge of the map (its own row/col is 0 or rows-1/cols-1) also
    # counts as a boundary cell, since "the map ends here" is exactly the
    # same kind of discontinuity as "the region ends here" for this purpose.
    not_walkable = ~walkable_mask
    boundary |= walkable_mask & np.pad(not_walkable[:-1, :], ((1, 0), (0, 0)), constant_values=True)
    boundary |= walkable_mask & np.pad(not_walkable[1:, :], ((0, 1), (0, 0)), constant_values=True)
    boundary |= walkable_mask & np.pad(not_walkable[:, :-1], ((0, 0), (1, 0)), constant_values=True)
    boundary |= walkable_mask & np.pad(not_walkable[:, 1:], ((0, 0), (0, 1)), constant_values=True)
    return boundary


def _line_cells(p1: tuple[int, int], p2: tuple[int, int]) -> list[tuple[int, int]]:
    """Bresenham's line algorithm - every grid cell on the straight line
    from p1 to p2 inclusive, in order. Used to walk the candidate connecting
    line between two components' nearest points and inspect every cell it
    actually crosses, the same kind of explicit, step-by-step grid walk
    `nav/prm_planner.py`'s own line-of-sight check already uses for the
    identical reason (need the exact cell sequence, not just a distance).
    """
    r1, c1 = p1
    r2, c2 = p2
    cells = []
    dr = abs(r2 - r1)
    dc = abs(c2 - c1)
    sr = 1 if r1 < r2 else -1
    sc = 1 if c1 < c2 else -1
    err = dr - dc
    r, c = r1, c1
    while True:
        cells.append((r, c))
        if r == r2 and c == c2:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells
