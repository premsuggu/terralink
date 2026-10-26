"""W1 - "House construction site": the pack author's own placed layout.

A 100 x 100 m fenced yard around a 55 x 45 m multi-storey building, two tower
cranes, dump/concrete trucks, container stacks, material piles, ramps and a
surrounding road. Realism comes straight from the CC0 Majadroid pack; nothing
is hand-placed here. What it stresses: long routes (~100 m across), a huge
solid obstacle in the middle, scattered clutter of every size, and a map ~39x
larger than the 16 m crop the A* planner was profiled on.
"""
from __future__ import annotations

from ..assets import Pack, export_model
from ..scene import Scene

NAME = "w1_house_site"
TITLE = "W1  House construction site (CC0 Majadroid layout)"
CEILING = 3.0       # surfaces whose lowest point is above this are "overhead" (crane jibs, upper storeys) and ignored
UAV_CEILING = 5.0   # UAV-view variant: just above the live patrol altitude (3.0-4.5 m in tunnel_demo/nav_sim), so decks are seen, jibs are not

CAM_PX = 2000       # run-video camera resolution: the map is 120 m wide, so more pixels than the 60 m worlds

# Scenario markers (start + goals), checked on the UAV-view map with replay_plan.py:
#   A  far corner of the south yard: 105 m route around the building (detour x1.22) - the long-route test
#   B  inside the building, on the first slab: its ring wall is closed, so "no path" is the right answer
#   C  outside the double fence (west): unreachable, "no path" is the right answer
MARKERS = [
    ("S", (-40.0, 5.0), "s", "#2ca02c"),
    ("A", (30.0, -45.0), "*", "#d62728"),
    ("B", (20.0, -20.0), "*", "#d62728"),
    ("C", (-56.0, 0.0), "*", "#d62728"),
]


def build(pack: Pack, models_dir) -> Scene:
    parts = pack.load_all_in_one()                      # {'roads': Asset, 'houseconstructionsite': Asset}
    scene = Scene(NAME, bounds=(-60.0, 60.0, -60.0, 60.0), ground_rgb=(0.35, 0.37, 0.33), ground_z=-0.30)
    for slug, asset in parts.items():
        model = export_model(models_dir, pack, asset, desc=f"W1 {slug} (pack layout, world coordinates)")
        scene.add_placed_asset(asset, model)
    return scene
