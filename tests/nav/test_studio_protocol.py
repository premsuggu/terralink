"""Studio wire format: the binary map frame and the JSON helper (no ROS, no server)."""
import json

import numpy as np
import pytest

from nav.studio import protocol


def _maps(rows=40, cols=50, seed=0):
    rng = np.random.default_rng(seed)
    elev = rng.uniform(-1, 3, (rows, cols)).astype(np.float32)
    valid = np.zeros((rows, cols), dtype=bool)
    valid[10:25, 12:30] = True                      # an observed block in the middle
    walk = rng.random((rows, cols)) > 0.4           # random "walkable" everywhere, including unobserved cells
    return elev, valid, walk


class TestMapFrame:
    def test_round_trip_keeps_observed_cells_to_one_millimetre(self):
        elev, valid, walk = _maps()
        pm = protocol.unpack_map(protocol.pack_map(elev, valid, walk, 0.1, 1.5, -2.0, version=7))
        full = pm.full()
        assert np.array_equal(full["valid"], valid)
        assert np.nanmax(np.abs(full["elevation"][valid] - elev[valid])) <= 0.0006        # int16 millimetres
        assert np.isnan(full["elevation"][~valid]).all()                                  # unobserved -> NaN

    def test_a_cell_nobody_has_seen_is_never_walkable(self):
        elev, valid, walk = _maps()
        full = protocol.unpack_map(protocol.pack_map(elev, valid, walk, 0.1, 0, 0, 1)).full()
        assert not full["walkable"][~valid].any()
        assert np.array_equal(full["walkable"][valid], walk[valid])

    def test_only_the_observed_bounding_box_plus_margin_is_sent(self):
        elev, valid, walk = _maps()
        pm = protocol.unpack_map(protocol.pack_map(elev, valid, walk, 0.1, 0, 0, 1, margin=2))
        h = pm.header
        assert (h["r0"], h["c0"], h["h"], h["w"]) == (8, 10, 19, 22)       # rows 10..24 and cols 12..29, plus 2 each side
        assert (h["rows"], h["cols"]) == (40, 50)

    def test_margin_never_leaves_the_grid(self):
        elev = np.zeros((10, 10), dtype=np.float32)
        valid = np.ones((10, 10), dtype=bool)
        h = protocol.unpack_map(protocol.pack_map(elev, valid, valid, 0.1, 0, 0, 1, margin=5)).header
        assert (h["r0"], h["c0"], h["h"], h["w"]) == (0, 0, 10, 10)

    def test_an_empty_map_is_a_valid_empty_frame(self):
        z = np.zeros((20, 20), dtype=np.float32)
        pm = protocol.unpack_map(protocol.pack_map(z, np.zeros((20, 20), bool), np.ones((20, 20), bool), 0.1, 0, 0, 3))
        assert pm.header["h"] == 0 and pm.header["w"] == 0
        assert not pm.full()["valid"].any()

    def test_geometry_and_version_are_in_the_header(self):
        elev, valid, walk = _maps()
        h = protocol.unpack_map(protocol.pack_map(elev, valid, walk, 0.1, 1.5, -2.0, version=42)).header
        assert (h["v"], h["res"], h["cx"], h["cy"]) == (42, 0.1, 1.5, -2.0)

    def test_heights_beyond_the_int16_range_are_clipped_not_wrapped(self):
        elev = np.full((4, 4), 500.0, dtype=np.float32)       # 500 m would overflow int16 millimetres
        valid = np.ones((4, 4), dtype=bool)
        full = protocol.unpack_map(protocol.pack_map(elev, valid, valid, 0.1, 0, 0, 1, margin=0)).full()
        assert np.allclose(full["elevation"], 32.767)

    def test_nan_elevation_in_an_observed_cell_does_not_crash(self):
        elev = np.full((4, 4), np.nan, dtype=np.float32)
        valid = np.ones((4, 4), dtype=bool)
        assert protocol.unpack_map(protocol.pack_map(elev, valid, valid, 0.1, 0, 0, 1)).valid.all()

    @pytest.mark.parametrize("bad", [b"", b"\x02abcd", b"\x01\x00", b"\x01\xff\xff\xff\xffxx"])
    def test_malformed_frames_are_rejected(self, bad):
        with pytest.raises(ValueError):
            protocol.unpack_map(bad)

    def test_a_truncated_body_is_rejected(self):
        elev, valid, walk = _maps()
        frame = protocol.pack_map(elev, valid, walk, 0.1, 0, 0, 1)
        with pytest.raises(ValueError):
            protocol.unpack_map(frame[:-5])

    def test_a_full_w2_sized_map_stays_below_one_megabyte(self):
        n = 560
        elev = np.zeros((n, n), dtype=np.float32)
        valid = np.ones((n, n), dtype=bool)
        assert len(protocol.pack_map(elev, valid, valid, 0.1, 0, 0, 1)) < 1_000_000


class TestJson:
    def test_nan_and_inf_become_null(self):
        out = json.loads(protocol.dumps({"a": float("nan"), "b": [1.0, float("inf")], "c": {"d": float("-inf")}}))
        assert out == {"a": None, "b": [1.0, None], "c": {"d": None}}

    def test_numpy_scalars_are_accepted(self):
        out = json.loads(protocol.dumps({"x": np.float32(1.5), "n": np.int64(3), "bad": np.float64("nan")}))
        assert out == {"x": 1.5, "n": 3, "bad": None}

    def test_output_is_compact(self):
        assert protocol.dumps({"t": "state", "x": 1}) == '{"t":"state","x":1}'
