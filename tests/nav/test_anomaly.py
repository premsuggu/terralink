"""Unit tests for nav.anomaly.compute_anomaly_mask - the second version of
the connected-component gap-detection redesign (step06 anomaly-detection-v2,
second fix).

The FIRST redesign (component topology + KD-tree distance/thickness, no
elevation check) fixed the original local heuristic's "random-looking
flags" problem, but introduced a new, more serious bug found live during
its OWN verification: two genuine rooms separated only by an ordinary thin
wall (no tunnel, no island between them) got flagged as if they were a
real occluded passage - because pure topology plus thickness cannot
structurally distinguish "real wall" from "real occluded gap" at all; both
produce an identical "two large, close, disconnected regions with a thin
LETHAL band between them" signature.

This version restores an elevation-based discriminator (the original
heuristic's own idea) but applies it to whole COMPONENTS in a specific
three-way room-island-room role split, not per-scan-line: an island must
sit BETWEEN at least two real rooms, close and thinly-connected to each,
AND at a measurably different elevation than each - the one structural
signature a real occluded passage has that an ordinary wall never does
(nothing sits between the two rooms a wall separates).

Room sizes throughout are >= 4.0 m² (well above the default
min_room_area_m2=3.0), island sizes are 0.2-0.4 m² (within the default
[0.15, 3.0) m² range and close to this project's real, live-measured
tunnel interior, ~0.37 m²), and robot-artifact-sized slivers are ~0.09 m²
(matching nav_ugv's real chassis footprint) - all calibrated against the
real numbers found live, not arbitrary synthetic values.
"""
import numpy as np

from nav.anomaly import compute_anomaly_mask

RESOLUTION = 0.1  # meters/cell, matching this project's worlds


def _grid(rows: int, cols: int, elevation: float = 0.0) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A fully-observed grid to start building a test scene from
    (elevation=uniform baseline, walkable=False, is_valid=True everywhere) -
    callers carve out walkable regions and set elevations on top of this.
    """
    elev = np.full((rows, cols), elevation, dtype=np.float32)
    walkable = np.zeros((rows, cols), dtype=bool)
    is_valid = np.ones((rows, cols), dtype=bool)
    return elev, walkable, is_valid


class TestGenuineTunnelGap:
    def test_island_between_two_rooms_at_different_elevation_is_flagged(self):
        # Room A (elevation 0.0, 20x20=4.0 m²) -- thin LETHAL band -- island
        # (elevation 0.7, 6x6=0.36 m², matching the real tunnel interior's
        # own live-measured size almost exactly) -- thin LETHAL band --
        # Room B (elevation 0.0, 20x20=4.0 m²). The real tunnel signature.
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True     # room A
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True    # island (6x6)
        elev[9:15, 22:28] = 0.7
        walkable[2:22, 30:50] = True    # room B
        elev[2:22, 30:50] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)

        assert anomaly.any()
        # Both connecting bands (cols 20-21 and 28-29, within the island's
        # own row range) must be flagged...
        assert anomaly[9:15, 20:22].any()
        assert anomaly[9:15, 28:30].any()
        # ...but never the island itself, or either room's own interior.
        assert not anomaly[9:15, 22:28].any()
        assert not anomaly[2:22, 0:20].any()
        assert not anomaly[2:22, 30:50].any()


class TestThinWallRegressionFromV2FirstFix:
    """REAL BUG FOUND LIVE (see compute_anomaly_mask's own module
    docstring): the first anomaly-detection-v2 redesign flagged two
    genuinely separate rooms connected by nothing but an ordinary thin
    wall - this project's walls are all 0.15m (1-2 cells), well within any
    reasonable thickness cap, so a plain distance+thickness check cannot
    tell this apart from a real tunnel. This is THE regression test that
    must never silently start failing again.
    """

    def test_two_large_rooms_across_a_thin_wall_with_no_island_is_not_flagged(self):
        # Directly reproduces the live-found failure: two 13+ m² rooms
        # separated by a UNIFORM 2-cell (0.2m) gap along their entire
        # shared boundary - no island anywhere. A real wall.
        elev, walkable, is_valid = _grid(80, 40)
        walkable[5:75, 0:19] = True    # room A, x cols 0-18 (14.0 m²)
        elev[5:75, 0:19] = 0.0
        walkable[5:75, 21:40] = True   # room B, x cols 21-39 (13.3 m²)
        elev[5:75, 21:40] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()

    def test_same_scene_with_a_real_elevation_difference_still_not_flagged_without_an_island(self):
        # Confirms the fix isn't just "the elevations happened to match" -
        # even with genuinely different elevations on either side, no
        # flag fires without an actual island component present. A real
        # wall between a room and a raised platform is still just a wall.
        elev, walkable, is_valid = _grid(80, 40)
        walkable[5:75, 0:19] = True
        elev[5:75, 0:19] = 0.0
        walkable[5:75, 21:40] = True
        elev[5:75, 21:40] = 0.7  # a real elevation difference this time

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()


class TestRobotArtifactRegressionFromV1Fix:
    """REAL BUG FOUND LIVE (the original v1 fix - see module docstring): a
    parked nav_ugv's own chassis (~0.09 m²) reading as a tiny, spuriously
    "disconnected" walkable sliver next to real floor. Confirms this stays
    fixed under the new room/island size split.
    """

    def test_robot_sized_sliver_between_two_rooms_is_not_treated_as_an_island(self):
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[10:13, 23:26] = True   # 3x3 = 0.09 m², a robot-chassis-sized sliver
        elev[10:13, 23:26] = 0.7        # even at a different elevation - size alone excludes it
        walkable[2:22, 29:49] = True
        elev[2:22, 29:49] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()


class TestElevationDiscriminator:
    def test_island_at_the_same_elevation_as_both_rooms_is_not_flagged(self):
        # Same room-island-room topology as the genuine tunnel case, but
        # the "island" is at the SAME elevation as both rooms - an
        # ordinary, single-height floor that happens to read as three
        # components for some unrelated reason, not a real overhang.
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.0  # same as both rooms
        walkable[2:22, 30:50] = True
        elev[2:22, 30:50] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()

    def test_elevation_difference_below_threshold_is_not_flagged(self):
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.05  # well under the default 0.15m threshold
        walkable[2:22, 30:50] = True
        elev[2:22, 30:50] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()

    def test_island_differing_from_only_one_room_is_not_flagged(self):
        # The island must differ from BOTH rooms, not just one - a
        # plausible case if one "room" is actually itself already at the
        # island's own elevation for some unrelated reason.
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.7
        walkable[2:22, 30:50] = True
        elev[2:22, 30:50] = 0.7  # matches the island - no real difference here

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        # Only the A-island connection could ever qualify (differs from A),
        # but a single qualifying neighbor is never enough on its own (see
        # TestRequiresAtLeastTwoRoomNeighbors) - either way, must not flag.
        assert not anomaly.any()


class TestRequiresAtLeastTwoRoomNeighbors:
    def test_island_with_only_one_room_neighbor_is_not_flagged(self):
        # A dead-end island - close to exactly one real room, nothing on
        # the other side (e.g. a genuine alcove, or simply not enough of
        # the map scanned yet to know). One qualifying connection alone
        # isn't evidence of a through-passage.
        elev, walkable, is_valid = _grid(24, 30)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.7
        # No second room on the far side at all.

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()

    def test_island_between_three_rooms_flags_every_qualifying_connection(self):
        # A junction-shaped island with THREE real room neighbors, all at a
        # different elevation - every qualifying connection should be
        # flagged, not just the first two found. Room C is sized to 4.0 m²
        # like A and B (>= min_room_area_m2) - an earlier version of this
        # test mistakenly used a 1.8 m² room C, which fell below the room
        # threshold and was silently treated as a second, unrelated ISLAND
        # instead of a room - a real lesson in getting test geometry right
        # against the actual thresholds, not the algorithm's own fault.
        elev, walkable, is_valid = _grid(55, 50)
        walkable[2:22, 0:20] = True      # room A (west)
        elev[2:22, 0:20] = 0.0
        walkable[22:28, 22:28] = True    # island (center, 6x6)
        elev[22:28, 22:28] = 0.7
        walkable[2:22, 30:50] = True     # room B (east)
        elev[2:22, 30:50] = 0.0
        walkable[30:50, 15:35] = True    # room C (south, 20x20 = 4.0 m²)
        elev[30:50, 15:35] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert anomaly[22:28, 20:22].any()  # island <-> room A
        assert anomaly[22:28, 28:30].any()  # island <-> room B
        assert anomaly[28:30, 22:28].any()  # island <-> room C


class TestDistanceAndThicknessGating:
    def test_island_too_far_from_a_room_is_not_flagged(self):
        elev, walkable, is_valid = _grid(24, 80)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 60:66] = True   # island, far from room A (4m away)
        elev[9:15, 60:66] = 0.7
        walkable[2:22, 68:78] = True   # room B, close to the island (0.2m)
        elev[2:22, 68:78] = 0.0

        anomaly = compute_anomaly_mask(
            elev, walkable, is_valid, RESOLUTION, max_gap_distance_m=1.5
        )
        # The island only ever gets ONE qualifying neighbor (room B) - room
        # A is too far to even be considered - so nothing is flagged.
        assert not anomaly.any()

    def test_thick_band_between_island_and_room_is_not_flagged(self):
        elev, walkable, is_valid = _grid(24, 60)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 30:36] = True   # island
        elev[9:15, 30:36] = 0.7
        walkable[2:22, 40:60] = True   # room B - but the gap to it is 4 cells (thick)
        elev[2:22, 40:60] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        # room A <-> island is thin (10 cells gap - wait this is checked
        # directly): only the island<->B connection is thick; A<->island
        # alone is a single qualifying neighbor, not enough on its own.
        assert not anomaly.any()

    def test_unobserved_connecting_line_is_not_flagged(self):
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.7
        walkable[2:22, 30:50] = True
        elev[2:22, 30:50] = 0.0
        is_valid[9:15, 28:30] = False  # the island-to-B connection was never scanned

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        # Only one confirmed-LETHAL qualifying connection (A<->island)
        # remains - not enough on its own.
        assert not anomaly.any()


class TestEdgeCases:
    def test_single_connected_component_flags_nothing(self):
        elev, walkable, is_valid = _grid(10, 10)
        walkable[:, :] = True
        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()

    def test_all_lethal_grid_flags_nothing(self):
        elev, walkable, is_valid = _grid(10, 10)
        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()
        assert anomaly.shape == (10, 10)

    def test_only_two_rooms_no_islands_flags_nothing(self):
        # Two rooms, no third component of any size - the exact shape of
        # "there is nothing between these two regions at all", the plainest
        # form of a real wall.
        elev, walkable, is_valid = _grid(24, 40)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[2:22, 22:40] = True
        elev[2:22, 22:40] = 0.0
        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        assert not anomaly.any()


class TestRealisticLayout:
    def test_2d_room_and_tunnel_layout_flags_only_the_tunnel_mouths(self):
        # A layout shaped like this project's real tunnel_test.world: two
        # large rooms joined only by a narrow, differently-elevated
        # corridor (the tunnel's own roof reading) - the real scenario
        # this whole module exists to get right.
        rows, cols = 30, 60
        elev, walkable, is_valid = _grid(rows, cols)
        walkable[2:28, 1:24] = True     # room A (23x26 cells, ~6 m²)
        elev[2:28, 1:24] = 0.0
        walkable[10:20, 26:31] = True   # the tunnel/corridor (10x5 cells, 0.5 m²)
        elev[10:20, 26:31] = 0.7
        walkable[2:28, 33:59] = True    # room B
        elev[2:28, 33:59] = 0.0
        # Cols 24-25 and 31-32 stay LETHAL (default) - the thin walls
        # separating the corridor from each room.

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)

        assert anomaly[10:20, 24:26].any()
        assert anomaly[10:20, 31:33].any()
        assert not anomaly[2:28, 1:20].any()
        assert not anomaly[2:28, 37:59].any()
        assert not anomaly[10:20, 26:31].any()  # the corridor itself never flagged


class TestThickenedBandStaysWithinLethalCells:
    def test_flagged_cells_are_always_genuinely_lethal(self):
        elev, walkable, is_valid = _grid(24, 50)
        walkable[2:22, 0:20] = True
        elev[2:22, 0:20] = 0.0
        walkable[9:15, 22:28] = True
        elev[9:15, 22:28] = 0.7
        walkable[2:22, 30:50] = True
        elev[2:22, 30:50] = 0.0

        anomaly = compute_anomaly_mask(elev, walkable, is_valid, RESOLUTION)
        lethal = is_valid & ~walkable
        assert anomaly.any()
        assert np.array_equal(anomaly & ~lethal, np.zeros_like(anomaly))
