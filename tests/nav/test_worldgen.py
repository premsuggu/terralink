"""Tests for the world-generation tooling in src/nav/scripts/worldgen/.

No ROS and no Gazebo are needed. Tests that use the CC0 Majadroid pack are
skipped when the pack folder is not present (it lives under tests/worlds/).

What is pinned here, and why:
  * Raster coverage is CONSERVATIVE - thin walls and vertical faces must reach
    the height grid, otherwise the "ground truth" the planner is judged
    against would silently contain the thin-wall-seam failure of step 08.
  * The grid geometry follows the nav snapshot convention (cell centre =
    (index - n/2) * res + centre), so the stored maps drop into
    replay_plan.py / map_replay_node unchanged.
  * Every generated world is valid SDF, only references models that exist,
    and the start/goal markers have the feasibility we designed them to have.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "nav" / "scripts"))

from worldgen.feasibility import RADII, reachability, to_cell     # noqa: E402
from worldgen.layout import rect_walls, seg_wall                  # noqa: E402
from worldgen.raster import Raster                                # noqa: E402
from worldgen.scene import Scene                                  # noqa: E402

PACK = REPO / "tests/worlds/gazebo_world_conversion/LowPoly-House-Construction-Site-By-Majadroid"
needs_pack = pytest.mark.skipif(not PACK.exists(), reason="Majadroid pack not present under tests/worlds/")


def _quad(p0, p1, p2, p3):
    """Two triangles for a quad (4 xyz corners)."""
    v = np.array([p0, p1, p2, p3], float)
    return v, np.array([[0, 1, 2], [0, 2, 3]])


# --------------------------------------------------------------------------- raster
class TestRasterCoverage:
    def test_flat_slab_height(self):
        R = Raster(0, 4, 0, 4, 0.1, oversample=3)
        v, t = _quad((1, 1, 0.5), (3, 1, 0.5), (3, 3, 0.5), (1, 3, 0.5))
        R.add(v, t, np.full((2, 3), 200, np.uint8))
        R.finish()
        assert R.height[20, 20] == pytest.approx(0.5)            # inside
        assert not np.isfinite(R.height[5, 5])                   # outside stays empty

    def test_wall_thinner_than_a_cell_is_not_dropped(self):
        """A 4 cm thick wall (< one 0.1 m cell) must still occupy its cells along the whole length."""
        R = Raster(0, 4, 0, 4, 0.1, oversample=4)
        # top face of a wall along x at y = 2.03 .. 2.07, height 1.5
        v, t = _quad((0.5, 2.03, 1.5), (3.5, 2.03, 1.5), (3.5, 2.07, 1.5), (0.5, 2.07, 1.5))
        R.add(v, t, np.full((2, 3), 100, np.uint8))
        R.finish()
        row = 20                                                  # y = 2.0..2.1
        assert np.all(R.height[row, 6:34] == pytest.approx(1.5)), "thin wall lost cells"

    def test_vertical_face_without_a_top_is_kept(self):
        """A wall modelled as one vertical quad (zero plan-view area) must appear at its top height."""
        R = Raster(0, 4, 0, 4, 0.1, oversample=4)
        v, t = _quad((0.5, 2.0, 0.0), (3.5, 2.0, 0.0), (3.5, 2.0, 2.0), (0.5, 2.0, 2.0))
        R.add(v, t, np.full((2, 3), 100, np.uint8))
        R.finish()
        cells = R.height[19:21, 6:34]
        assert np.all(cells.max(axis=0) == pytest.approx(2.0)), "vertical wall dropped"

    def test_ceiling_drops_hovering_slab_but_keeps_ground_wall(self):
        R = Raster(0, 4, 0, 4, 0.1, oversample=2)
        slab_v, slab_t = _quad((0, 0, 3.2), (2, 0, 3.2), (2, 2, 3.2), (0, 2, 3.2))     # hovering deck
        R.add(slab_v, slab_t, np.full((2, 3), 50, np.uint8), ceiling=3.0)
        # a tall wall starting at the ground: lowest point 0 < ceiling -> kept at full height
        wall_v, wall_t = _quad((2.5, 1.0, 0.0), (2.5, 3.0, 0.0), (2.5, 3.0, 4.0), (2.5, 1.0, 4.0))
        R.add(wall_v, wall_t, np.full((2, 3), 80, np.uint8), ceiling=3.0)
        R.finish()
        assert not np.isfinite(R.height[10, 10])                  # deck ignored
        assert np.nanmax(R.height[:, 24:26]) == pytest.approx(4.0)  # wall kept at 4 m


# --------------------------------------------------------------------------- scene / map
class TestSceneGroundTruth:
    def _scene(self):
        sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
        sc.add_box("blk", 0.0, 0.0, 2.0, 2.0, 1.0)               # 2x2x1 box at the origin
        return sc

    def test_box_is_a_lethal_ring_with_correct_geometry(self):
        layers, cx, cy, _ = self._scene().ground_truth(res=0.1, ceiling=3.0, oversample=2)
        elev, trav = layers["elevation"], layers["traversability"]
        res, n = 0.1, elev.shape[0]
        # nav snapshot convention: centre of cell (row, col) = ((col - n/2)*res + cx, (row - n/2)*res + cy)
        xs = (np.arange(n) - n / 2) * res + cx
        ys = (np.arange(n) - n / 2) * res + cy
        inside = (np.abs(xs[None, :]) < 0.9) & (np.abs(ys[:, None]) < 0.9)
        assert np.allclose(elev[inside], 1.0)
        assert np.allclose(elev[(np.abs(xs[None, :]) > 1.2) | (np.abs(ys[:, None]) > 1.2)], 0.0)
        r, c = to_cell(1.0, 0.0, n, n, res, cx, cy)               # the box edge
        assert trav[r, c - 1:c + 2].min() < 0.15                  # lethal band at the edge
        r0, c0 = to_cell(3.0, 3.0, n, n, res, cx, cy)
        assert trav[r0, c0] == 1.0                                # open ground is easy

    def test_world_sdf_is_valid_xml(self):
        root = ET.fromstring(self._scene().world_sdf())
        names = [m.get("name") for m in root.iter("model")]
        assert "ground_plane" in names and "blk" in names

    def test_exclude_kinds_removes_items_from_world_but_not_map(self):
        sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
        sc.add_box("deck", 0, 0, 4, 4, 0.4, z0=3.1, kind="overhead")
        assert "deck" in sc.world_sdf()
        assert "deck" not in sc.world_sdf(exclude_kinds=("overhead",))
        layers, *_ = sc.ground_truth(res=0.1, ceiling=3.0, oversample=2)
        assert layers["elevation"].max() == 0.0                   # hovering deck ignored by the ground-truth map
        uav, *_ = sc.ground_truth(res=0.1, ceiling=np.inf, oversample=2)
        assert uav["elevation"].max() == pytest.approx(3.5)       # but a UAV sees it as solid


# --------------------------------------------------------------------------- layout + feasibility
class TestLayoutAndFeasibility:
    def _gap_scene(self, gap):
        sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
        seg_wall(sc, "w", 0.0, -5.0, 0.0, 5.0, gaps=[(0.0, gap)], thick=0.3, h=2.0)
        return sc

    def test_seg_wall_makes_two_pieces_around_a_gap(self):
        sc = self._gap_scene(1.0)
        assert [i.name for i in sc.items] == ["w_0", "w_1"]

    def test_rect_walls_gate(self):
        sc = Scene("t", bounds=(-5, 5, -5, 5))
        rect_walls(sc, "f", -4, -4, 4, 4, gaps=[("S", 0.0, 2.0)])
        assert sum(1 for i in sc.items if i.name.startswith("f_S")) == 2
        assert sum(1 for i in sc.items if i.name.startswith("f_N")) == 1

    # The map's lethal band eats ~0.15 m per side (conservative raster edge + the 3x3 step test), so the
    # largest passable robot radius is about gap/2 - 0.15 m: a nominal 1.2 m door passes r=0.35 not r=0.5.
    @pytest.mark.parametrize("gap,radius,expected", [(1.2, 0.35, True), (1.2, 0.5, False), (0.9, 0.25, True),
                                                      (0.5, 0.05, True), (0.5, 0.25, False)])
    def test_reachability_through_a_doorway(self, gap, radius, expected):
        layers, cx, cy, _ = self._gap_scene(gap).ground_truth(res=0.1, ceiling=3.0, oversample=2)
        out = reachability(layers["traversability"], 0.1, cx, cy, (-3.0, 0.0), {"G": (3.0, 0.0)}, radii=(radius,))
        assert out["G"][radius] is expected

    def test_closed_wall_is_unreachable(self):
        sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
        seg_wall(sc, "w", 0.0, -5.2, 0.0, 5.2, thick=0.3, h=2.0)
        layers, cx, cy, _ = sc.ground_truth(res=0.1, ceiling=3.0, oversample=2)
        out = reachability(layers["traversability"], 0.1, cx, cy, (-3.0, 0.0), {"G": (3.0, 0.0)})
        assert not any(out["G"].values())


# --------------------------------------------------------------------------- pack + worlds
@needs_pack
class TestPackAndFbx:
    def test_all_in_one_extents_are_metres_and_z_up(self):
        from worldgen.assets import Pack

        parts = Pack(PACK).load_all_in_one()
        site = parts["houseconstructionsite"].mesh.vertices
        assert site[:, 0].min() == pytest.approx(-50.0, abs=0.1) and site[:, 0].max() == pytest.approx(50.0, abs=0.1)
        assert 60.0 < site[:, 2].max() < 80.0                     # a ~60 m building: Z is up

    def test_prop_library_is_centred_and_real_size(self):
        from worldgen.assets import Pack

        truck = Pack(PACK).assets["dumper_truck_red"]
        assert 6.5 < truck.footprint[1] < 7.5 and 2.8 < truck.footprint[0] < 3.5
        assert abs(truck.mesh.vertices[:, 0].mean()) < 0.5        # props are authored around the origin

    def test_fence_geometry_is_never_dropped_by_the_rasteriser(self):
        """Every fence vertex (incl. vertical panels) must be covered by a cell at least that tall."""
        from worldgen.assets import Pack

        fence = Pack(PACK).assets["fence"].mesh
        R = Raster(-60, 60, -60, 60, 0.1, oversample=3)
        R.add(fence.vertices, fence.triangles, np.zeros((len(fence.triangles), 3), np.uint8))
        R.finish()
        v = fence.vertices
        cols = ((v[:, 0] + 60) / 0.1).astype(int).clip(0, R.nx - 1)
        rows = ((v[:, 1] + 60) / 0.1).astype(int).clip(0, R.ny - 1)
        assert np.all(R.height[rows, cols] >= v[:, 2] - 0.02)


W1_MAP = REPO / "src/nav/maps/w1_house_site_groundtruth.npz"


@pytest.mark.skipif(not W1_MAP.exists(), reason="W1 ground-truth map not generated")
def test_w1_fence_ring_is_closed_except_the_packs_own_gates():
    """North/south fence must be a closed lethal line; east/west have only the pack's two openings."""
    d = np.load(W1_MAP)
    trav, res, cx, cy = d["layer_traversability"], float(d["resolution"]), float(d["center_x"]), float(d["center_y"])
    n = trav.shape[0]
    coords = (np.arange(n) - n / 2) * res + cx
    inner = (coords > -49.5) & (coords < 49.5)
    lethal = trav < 0.15

    def line(axis, v):
        i = int(np.argmin(np.abs(coords - v)))
        return lethal[:, i] if axis == "x" else lethal[i, :]

    def gap_len(prof):
        # best (most lethal) of the neighbouring rows/cols so a 1-cell shift of the fence cannot matter
        return (~prof[inner]).sum() * res

    north = np.maximum(line("y", 49.65), line("y", 49.75))
    south = np.maximum(line("y", -49.75), line("y", -49.85))
    assert gap_len(north) < 0.6 and gap_len(south) < 0.6
    east = np.maximum(line("x", 49.65), line("x", 49.75))
    west = np.maximum(line("x", -49.75), line("x", -49.65))
    assert 17.0 < gap_len(east) < 21.0, "east gate (~19 m) changed"
    assert 8.0 < gap_len(west) < 10.0, "west gate (~9 m) changed"


# Expected feasibility of each world's markers: label -> reachable at the SMALLEST / LARGEST radius.
DESIGN = {
    "w1_house_site": {"A": (True, True), "B": (False, False), "C": (False, False)},
    "w2_container_yard": {"A": (True, True), "B": (True, True), "C": (True, False), "G": (False, False),
                          "D": (True, True), "E": (False, False), "F": (True, True)},
    "w3_excavation_slopes": {"A": (True, True), "B": (False, False), "C": (True, True), "D": (True, True),
                             "E": (True, True), "F": (True, True)},
    "w4_structure_decks": {"A": (True, True), "B": (True, False), "C": (False, False), "D": (True, True),
                           "E": (True, True), "F": (True, True)},
}


@needs_pack
@pytest.mark.parametrize("world", sorted(DESIGN))
def test_generated_world_is_valid_and_matches_its_design(world, tmp_path):
    import importlib

    from worldgen.assets import Pack

    mod = importlib.import_module(f"worldgen.worlds.{world}")
    models = tmp_path / "models"
    scene = mod.build(Pack(PACK), models)

    # 1) valid SDF; every model:// include resolves to a folder with a mesh and its texture
    root = ET.fromstring(scene.world_sdf())
    for uri in root.iter("uri"):
        name = uri.text.split("model://")[1]
        folder = models / name
        assert (folder / "mesh.obj").exists() and (folder / "model.sdf").exists(), name
        assert list(folder.glob("*.png")), f"{name}: texture must sit next to the OBJ"
        assert "map_Kd" in (folder / "mesh.mtl").read_text()

    # 2) the markers have the feasibility the world was designed to have
    layers, cx, cy, _ = scene.ground_truth(0.1, mod.CEILING, 4)
    start = next(m for m in mod.MARKERS if m[0] == "S")[1]
    goals = {m[0]: m[1] for m in mod.MARKERS if m[0] != "S"}
    res = reachability(layers["traversability"], 0.1, cx, cy, start, goals)
    for label, (small, large) in DESIGN[world].items():
        assert res[label][RADII[0]] is small, f"{world} goal {label} at r={RADII[0]}"
        assert res[label][RADII[-1]] is large, f"{world} goal {label} at r={RADII[-1]}"


@needs_pack
def test_w1_world_file_is_valid_xml(tmp_path):
    from worldgen.assets import Pack
    from worldgen.worlds import w1_house_site

    scene = w1_house_site.build(Pack(PACK), tmp_path / "models")
    root = ET.fromstring(scene.world_sdf())
    assert {"roads", "houseconstructionsite"} <= {i.get("name") for i in root.iter("include") if i.get("name")} \
        or len(list(root.iter("include"))) == 2


# --------------------------------------------------------------------------- PLY export (for media/3d_maps/viewer.html)
def _read_ply(path):
    lines = Path(path).read_text().splitlines()
    end = lines.index("end_header")
    header, body = lines[:end], np.array([[float(v) for v in ln.split()] for ln in lines[end + 1:]])
    return header, body


def test_elevation_ply_uses_project_colours_and_snapshot_geometry(tmp_path):
    from worldgen.ply import write_elevation_ply

    sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
    sc.add_box("a", 1.0, 2.0, 1.0, 1.0, 1.5)
    layers, cx, cy, _ = sc.ground_truth(0.1, 3.0, 2)
    out = tmp_path / "t_elevation.ply"
    n = write_elevation_ply(layers, cx, cy, 0.1, out)
    header, body = _read_ply(out)
    assert n == len(body) == layers["elevation"].size                      # one point per cell (small grid: no reduction)
    assert "comment point_spacing 0.100" in header                         # the viewer sizes points from this
    # a point on the box top sits at the box's world position, at its height
    top = body[(np.abs(body[:, 0] - 1.0) < 0.06) & (np.abs(body[:, 1] - 2.0) < 0.06)]
    assert len(top) >= 1 and np.allclose(top[:, 2], 1.5)
    # colours follow nav.ply_export (green easy / red lethal) - the SAME convention for every world
    open_ground = body[(np.abs(body[:, 0] + 4.0) < 0.06) & (np.abs(body[:, 1] + 4.0) < 0.06)][0]
    assert tuple(open_ground[3:6]) == (60, 200, 60)
    assert (np.all(body[:, 3:6] == (220, 40, 40), axis=1)).any()           # the box edge is lethal red


def test_elevation_ply_block_reduction_keeps_world_coordinates(tmp_path):
    from worldgen.ply import write_elevation_ply

    sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0))
    layers, cx, cy, _ = sc.ground_truth(0.1, 3.0, 2)                        # 100 x 100 cells
    out = tmp_path / "t_small.ply"
    n = write_elevation_ply(layers, cx, cy, 0.1, out, max_cells=50)         # force 2x2 block reduction
    header, body = _read_ply(out)
    assert n == 2500 and "comment point_spacing 0.200" in header
    assert body[:, 0].min() == pytest.approx(-4.9 + 0.05 - 0.05, abs=0.11)   # still spans the same world extent
    assert body[:, 0].max() == pytest.approx(4.95, abs=0.11)


def test_world_html_embeds_the_real_meshes(tmp_path):
    """The HTML world view carries every triangle with its colour; decoding recovers the geometry to < 1 mm."""
    import base64
    import re
    from worldgen.html import write_world_html

    sc = Scene("t", bounds=(-5.0, 5.0, -5.0, 5.0), ground_rgb=(0.5, 0.5, 0.5), ground_z=-0.3)
    sc.add_box("a", 0, 0, 2, 2, 1.0, rgb=(1.0, 0.0, 0.0))                   # 12 triangles, pure red
    sc.add_box("b", 3, 3, 1, 1, 2.0, rgb=(0.0, 0.0, 1.0))
    out = tmp_path / "t_world.html"
    assert write_world_html(sc, "T", out) == 24
    txt = out.read_text()
    lo = np.array(re.search(r"LO = \[([^\]]*)\]", txt).group(1).split(","), float)
    span = np.array(re.search(r"SPAN = \[([^\]]*)\]", txt).group(1).split(","), float)
    q = np.frombuffer(base64.b64decode(re.search(r'q = new Uint16Array\(bytes\("([^"]*)"', txt).group(1)), "<u2").reshape(-1, 3)
    V = lo + q / 65535 * span
    ref = np.concatenate([it.verts for it in sc.items])
    assert np.abs(V - ref).max() < 1e-3
    rgb = np.frombuffer(base64.b64decode(re.search(r'rgb = bytes\("([^"]*)"', txt).group(1)), np.uint8).reshape(-1, 3)
    assert (rgb[:12] == (255, 0, 0)).all() and (rgb[12:] == (0, 0, 255)).all()      # true colours, no classification
    assert "PointsMaterial" not in txt                                          # real surfaces, not a point cloud


def test_run_world_adds_ugv_and_overhead_camera_covering_the_map(tmp_path):
    """`<w>_run.world` = the review world + the UGV at S + a straight-down camera that sees the whole map."""
    import math
    import xml.etree.ElementTree as ET
    sys.path.insert(0, str(REPO / "src/nav/scripts"))
    from make_run_video import camera_geometry
    from worldgen.run_world import run_blocks

    sc = Scene("t", bounds=(-30.0, 30.0, -30.0, 30.0))
    f = tmp_path / "t_run.world"
    f.write_text(sc.world_sdf(extra_models=run_blocks((-3.0, -26.0, 0.0), 0.0, sc.bounds, px=1000, height=150.0)))
    root = ET.parse(f).getroot()                                        # valid XML
    names = [m.get("name") for m in root.iter() if m.tag in ("model", "include")] + [n.text for n in root.iter("name")]
    assert "nav_ugv" in names and "overhead_overview_camera" in names
    cx, cy, mpp = camera_geometry(f, 1000)
    assert (cx, cy) == (0.0, 0.0)
    assert 1000 * mpp >= 60.0 and 1000 * mpp < 64.0                       # footprint covers the 60 m map plus a margin
    assert math.isclose(mpp, 2 * 150.0 * math.tan(0.5 * 2 * math.atan(31.0 / 150.0)) / 1000, rel_tol=1e-3)
