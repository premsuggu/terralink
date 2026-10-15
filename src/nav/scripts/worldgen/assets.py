"""Turn the CC0 Majadroid construction-site pack into Gazebo-loadable models.

Pipeline per mesh part:  FBX  --(fbx.py)-->  Mesh  --(here)-->  OBJ+MTL+PNG
placed in `src/nav/models/site_<slug>/` together with a model.config and a
static model.sdf, so a world can `<include>` it by `model://site_<slug>`.

Why OBJ (not FBX/DAE): Ignition Fortress's mesh loader reads OBJ/DAE/STL, not
FBX, and step 12 of emap taught us the texture file must sit in the SAME
folder as the mesh and be referenced by a bare filename (MTL `map_Kd`).

The pack colours everything from one 256x256 palette PNG via UVs, so the
review images can recover each triangle's colour by sampling that PNG at the
triangle's UV centroid (no lighting - a flat "paint" look).
"""
from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .fbx import Mesh, load_meshes

PALETTE_NAME = "ImphenziaPalette01-256-Gradient.png"


def slugify(name: str) -> str:
    return "".join(ch.lower() if ch.isalnum() else "_" for ch in name).strip("_").replace("__", "_")


@dataclass
class Asset:
    slug: str
    mesh: Mesh
    colours: np.ndarray            # (T,3) uint8 per-triangle colour for the review images

    @property
    def size(self) -> np.ndarray:
        return self.mesh.vertices.max(0) - self.mesh.vertices.min(0)

    @property
    def footprint(self) -> tuple[float, float]:
        s = self.size
        return float(s[0]), float(s[1])


class Pack:
    """All parts of the pack, keyed by slug (e.g. `container_red`, `dumper_truck_red`)."""

    def __init__(self, pack_dir: str | Path):
        self.dir = Path(pack_dir)
        self.palette = np.asarray(Image.open(self.dir / PALETTE_NAME).convert("RGB"))
        self.assets: dict[str, Asset] = {}
        fbx_dir = self.dir / "fbx files"
        for f in sorted(fbx_dir.glob("*.fbx")):
            if f.name.startswith("_"):          # the all-in-one scene is loaded separately by name
                continue
            for m in load_meshes(str(f)):
                slug = slugify(m.name)
                self.assets[slug] = Asset(slug, m, self.colours_for(m))

    def load_all_in_one(self) -> dict[str, Asset]:
        """The pack author's own placed layout: {'roads': ..., 'housesite': ...}."""
        f = self.dir / "fbx files" / "_HouseConstructionSite-AllInOneObject_.fbx"
        return {slugify(m.name): Asset(slugify(m.name), m, self.colours_for(m)) for m in load_meshes(str(f))}

    def colours_for(self, m: Mesh) -> np.ndarray:
        if m.uvs is None:
            return np.full((len(m.triangles), 3), 128, np.uint8)
        h, w = self.palette.shape[:2]
        uv = m.uvs.mean(axis=1)                       # triangle's UV centroid
        u = np.clip((uv[:, 0] * w).astype(int), 0, w - 1)
        v = np.clip(((1.0 - uv[:, 1]) * h).astype(int), 0, h - 1)
        return self.palette[v, u]


# --------------------------------------------------------------------------- #
# Writing a model folder Gazebo can load
# --------------------------------------------------------------------------- #
def write_obj(path: Path, mesh: Mesh, mtl_name: str) -> None:
    """Flat-shaded OBJ: one normal per triangle, per-corner UVs, one material."""
    V, T = mesh.vertices, mesh.triangles
    n = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    lines = [f"mtllib {mtl_name}", "usemtl palette"]
    lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in V]
    if mesh.uvs is not None:
        lines += [f"vt {u:.5f} {v:.5f}" for u, v in mesh.uvs.reshape(-1, 2)]
    lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in n]
    for i, tri in enumerate(T):
        a, b, c = (int(k) + 1 for k in tri)
        if mesh.uvs is not None:
            ta, tb, tc = 3 * i + 1, 3 * i + 2, 3 * i + 3
            lines.append(f"f {a}/{ta}/{i + 1} {b}/{tb}/{i + 1} {c}/{tc}/{i + 1}")
        else:
            lines.append(f"f {a}//{i + 1} {b}//{i + 1} {c}//{i + 1}")
    path.write_text("\n".join(lines) + "\n")


_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><mesh><uri>model://{name}/mesh.obj</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>model://{name}/mesh.obj</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

_MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Majadroid (CC0), converted for Gazebo by TerraLink</name><email></email></author>
  <description>{desc}</description>
</model>
"""


def export_mesh_model(models_dir: Path, name: str, mesh: Mesh, texture: Path, desc: str = "") -> str:
    """Write `<name>/` (OBJ + MTL + texture + model.sdf/config) and return the model name.

    The texture is copied next to the OBJ and referenced by bare filename (see
    the module docstring for why). Collision = the same mesh (static model).
    """
    d = models_dir / name
    d.mkdir(parents=True, exist_ok=True)
    write_obj(d / "mesh.obj", mesh, "mesh.mtl")
    (d / "mesh.mtl").write_text(f"newmtl palette\nKa 1 1 1\nKd 1 1 1\nKs 0.05 0.05 0.05\nmap_Kd {texture.name}\n")
    if texture.resolve() != (d / texture.name).resolve():
        shutil.copy(texture, d / texture.name)
    (d / "model.sdf").write_text(_MODEL_SDF.format(name=name))
    (d / "model.config").write_text(_MODEL_CONFIG.format(name=name, desc=desc or name))
    return name


def export_model(models_dir: Path, pack: Pack, asset: Asset, desc: str = "") -> str:
    """Export one pack part as `site_<slug>` (see export_mesh_model)."""
    return export_mesh_model(models_dir, f"site_{asset.slug}", asset.mesh, pack.dir / PALETTE_NAME,
                             desc or f"{asset.slug} (Majadroid CC0 pack)")
