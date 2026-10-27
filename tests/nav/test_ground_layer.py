"""nav.ground_layer.combine_ground: the UGV's ground layer overrides the UAV's class where (and only where) it has data."""
import numpy as np

from emap.traversability import DIFFICULT, EASY, LETHAL
from nav.ground_layer import combine_ground
from nav.replay import MapSnapshot, build_masks


def _layers(n=6):
    return {"elevation": np.full((n, n), 3.5, np.float32), "traversability": np.full((n, n), EASY, np.float32),
            "is_valid": np.ones((n, n), np.float32), "variance": np.full((n, n), 0.1, np.float32)}


def test_without_ground_layers_it_is_the_identity():
    layers = _layers()
    assert combine_ground(layers) is layers


def test_ground_wins_where_it_is_valid_and_nowhere_else():
    layers = _layers()
    layers["is_valid"][0, :] = 0.0                                   # the UAV never saw row 0 (under a roof)
    gv = np.zeros((6, 6), np.float32); gv[0:2, 0:3] = 1.0
    layers["ground_valid"] = gv
    layers["ground_elevation"] = np.zeros((6, 6), np.float32)         # the floor under the 3.5 m deck
    gt = np.full((6, 6), EASY, np.float32); gt[1, 1] = LETHAL; gt[0, 2] = DIFFICULT
    layers["ground_traversability"] = gt
    out = combine_ground(layers)
    assert out["traversability"][1, 1] == LETHAL and out["traversability"][0, 2] == np.float32(DIFFICULT)   # the floor's class, not the deck's
    assert out["traversability"][3, 3] == EASY                       # no ground data: the UAV's class
    assert np.all(out["is_valid"][0, 0:3] == 1.0) and np.all(out["is_valid"][0, 3:] == 0.0)    # observed where the UGV saw, unchanged elsewhere
    assert out["elevation"][0, 0] == 0.0 and out["elevation"][3, 3] == 3.5
    assert layers["traversability"][1, 1] == EASY and layers["is_valid"][0, 0] == 0.0           # the inputs are untouched
    assert out["traversability"].dtype == layers["traversability"].dtype


def test_planner_masks_use_the_ground_layer():
    """A cell the UAV never observed becomes walkable once the UGV has measured it; a deck cell above a floor wall becomes blocked."""
    layers = _layers(8)
    layers["is_valid"][:, 4:] = 0.0                                   # right half unobserved by the UAV
    gv = np.zeros((8, 8), np.float32); gv[3, 4:7] = 1.0; gv[5, 1] = 1.0
    gt = np.full((8, 8), EASY, np.float32); gt[5, 1] = LETHAL         # a wall on the floor under the (flat, EASY) deck at (5, 1)
    layers.update(ground_valid=gv, ground_elevation=np.zeros((8, 8), np.float32), ground_traversability=gt)
    masks = build_masks(MapSnapshot(layers=layers, resolution=0.1, center_x=0.0, center_y=0.0))
    assert masks.walkable[3, 4:7].all() and not masks.walkable[2, 4:7].any()
    assert not masks.walkable[5, 1] and masks.walkable[5, 2]
