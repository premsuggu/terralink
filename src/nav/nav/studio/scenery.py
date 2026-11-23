"""The world's solid things (containers, fences, walls, crates) as plain boxes, so the browser can DRAW the world from the first moment.

Why this exists: in "live scan" mode the elevation map only contains what the UAV has already seen, so a freshly started page would show an
empty floor - the viewer would look broken. The site itself is known to the simulator (it is a Gazebo world file), so the server reads that file
once and sends the obstacles as a short list of boxes in the `hello` message. The page draws them as a faint "scenery" layer that can be switched
off; the robots do NOT know about it - they still only see what they scan - and the page keeps the two visibly different.

Reads the SDF world file (XML) of the studio world:
  * `<model>` with box visuals  -> one box per visual (size, colour from the material; a model's link/visual poses are identity in the
    generated worlds, and are ignored)
  * `<include>` of a mesh model -> one box = the axis-aligned bounding box of the model's `mesh.obj` (the cargo containers), placed with the
    include's pose; colour guessed from the model name (blue / white / green / red)
The robots (`iris_quad`, `nav_ugv`), the ground plane and the overhead camera are skipped.

Output item:  {"x","y","z": box centre in the map frame [m], "roll","pitch","yaw": rad, "sx","sy","sz": full sizes [m], "rgb": [r,g,b] 0..1}
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

_SKIP = {"ground_plane", "iris_quad", "nav_ugv", "overhead_overview_camera"}
_MAX_ITEMS = 3000                      # a sanity cap so a huge world file can never produce an enormous hello message
_COLOURS = {"blue": (0.22, 0.42, 0.70), "white": (0.88, 0.88, 0.85), "green": (0.25, 0.55, 0.32), "red": (0.70, 0.25, 0.20)}
_DEFAULT_RGB = (0.62, 0.62, 0.60)


def _floats(text: str | None, n: int) -> list[float]:
    """The first `n` numbers of a whitespace-separated string, padded with zeros."""
    vals = [float(t) for t in (text or "").split()[:n]]
    return vals + [0.0] * (n - len(vals))


def _mesh_bbox(obj_path: Path):
    """(min_xyz, max_xyz) of the vertices ('v x y z' lines) in a Wavefront .obj file, or None if unreadable."""
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    try:
        with open(obj_path) as f:
            for line in f:
                if line.startswith("v "):
                    p = [float(t) for t in line.split()[1:4]]
                    for i in range(3):
                        lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    except (OSError, ValueError):
        return None
    return (lo, hi) if lo[0] != math.inf else None


def load_scenery(world_file: str | Path, models_dir: str | Path) -> list[dict]:
    """Boxes describing the world file's solid things. Returns [] (never raises) if the file is missing or unreadable."""
    try:
        root = ET.parse(str(world_file)).getroot()
    except (OSError, ET.ParseError):
        return []
    world = root.find("world")
    if world is None:
        return []
    items: list[dict] = []
    bbox_cache: dict[str, object] = {}
    for node in world:
        if len(items) >= _MAX_ITEMS:
            break
        name = node.get("name") or (node.findtext("name") or "")
        if name in _SKIP:
            continue
        if node.tag == "model":
            x, y, z, roll, pitch, yaw = _floats(node.findtext("pose"), 6)
            for box in node.iter("box"):
                # only boxes inside a <visual> are drawn (collision boxes are identical in the generated worlds)
                size = _floats(box.findtext("size"), 3)
                if min(size) <= 0.0:
                    continue
                rgb = _DEFAULT_RGB
                vis = next((v for v in node.iter("visual") if box in list(v.iter("box"))), None)
                if vis is None:
                    continue
                amb = vis.findtext("material/diffuse") or vis.findtext("material/ambient")
                if amb:
                    rgb = tuple(_floats(amb, 3))
                items.append({"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw,
                              "sx": size[0], "sy": size[1], "sz": size[2], "rgb": [round(c, 3) for c in rgb]})
        elif node.tag == "include":
            uri = node.findtext("uri") or ""
            model = uri.replace("model://", "").strip("/")
            if not model:
                continue
            if model not in bbox_cache:
                bbox_cache[model] = _mesh_bbox(Path(models_dir) / model / "mesh.obj")
            bb = bbox_cache[model]
            if bb is None:
                continue
            lo, hi = bb
            x, y, z, _roll, _pitch, yaw = _floats(node.findtext("pose"), 6)
            # box centre in the model frame, rotated by the include's yaw (includes in these worlds are only yawed)
            ox, oy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
            c, s = math.cos(yaw), math.sin(yaw)
            rgb = next((v for k, v in _COLOURS.items() if k in model), _DEFAULT_RGB)
            items.append({"x": x + c * ox - s * oy, "y": y + s * ox + c * oy, "z": z + (lo[2] + hi[2]) / 2.0,
                          "roll": 0.0, "pitch": 0.0, "yaw": yaw,
                          "sx": hi[0] - lo[0], "sy": hi[1] - lo[1], "sz": hi[2] - lo[2], "rgb": list(rgb)})
    for it in items:                   # tidy numbers (the hello message is sent to every new tab)
        for k in ("x", "y", "z", "roll", "pitch", "yaw", "sx", "sy", "sz"):
            it[k] = round(it[k], 3)
    return items
