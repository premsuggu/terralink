"""A world described once, written out three consistent ways.

`Scene` collects placed things (pack meshes, boxes, ...). From that ONE
description it produces:
  1. the Ignition `.world` file            (`world_sdf`)
  2. the ground-truth 2.5D map             (`ground_truth`, a nav snapshot .npz)
  3. review images                         (`worldgen.images`)
so the map and the simulated world cannot drift apart.

Ground-truth meaning: the map is what an IDEAL overhead depth sensor would
report, except that surfaces hovering at/above `ceiling` metres are dropped
(a crane jib 50 m up is not a wall for a ground robot). Traversability is
computed by the project's own `emap.traversability` with the SAME thresholds
the live elevation node uses (max_slope 0.35, max_step 0.15, max_roughness
0.05, resolution 0.1), not a re-implementation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .assets import Asset, export_mesh_model
from .fbx import Mesh
from .raster import Raster

# Same thresholds as src/emap/config/elevation_mapping.yaml.
MAX_SLOPE, MAX_STEP, MAX_ROUGHNESS = 0.35, 0.15, 0.05
# Variance of a confidently-measured cell (well below MAX_ROUGHNESS=0.05).
CONFIDENT_VARIANCE = 0.0025


def _yaw_matrix(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@dataclass
class Item:
    name: str
    sdf: str                       # SDF snippet placed inside <world>
    verts: np.ndarray              # world-frame vertices (V,3)
    tris: np.ndarray               # (T,3)
    colours: np.ndarray            # (T,3) uint8
    kind: str = "prop"             # prop | ground | structure (used for image legends)


_BOX_FACES = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                       [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])


@dataclass
class Scene:
    name: str
    bounds: tuple[float, float, float, float]            # x_min, x_max, y_min, y_max of the MAP (metres)
    ground_rgb: tuple[float, float, float] = (0.45, 0.50, 0.30)
    ground_size: float = 400.0
    ground_z: float = 0.0           # height of the bare-ground plane (W1's roads sit at -0.2)
    items: list[Item] = field(default_factory=list)

    # ------------------------------------------------------------ placing --
    def add_asset(self, asset: Asset, model: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0,
                  name: str | None = None, kind: str = "prop") -> None:
        """Place a pack mesh (already exported as model://<model>) at a pose."""
        name = name or f"{asset.slug}_{len(self.items)}"
        R = _yaw_matrix(yaw)
        v = asset.mesh.vertices @ R.T + np.array([x, y, z])
        sdf = (f'<include><name>{name}</name><uri>model://{model}</uri>'
               f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.5f}</pose></include>')
        self.items.append(Item(name, sdf, v, asset.mesh.triangles, asset.colours, kind))

    def add_placed_asset(self, asset: Asset, model: str, kind: str = "structure") -> None:
        """Place a mesh whose vertices are already in world coordinates (pose = identity)."""
        self.add_asset(asset, model, 0, 0, 0, 0, name=asset.slug, kind=kind)

    def add_box(self, name: str, cx: float, cy: float, sx: float, sy: float, sz: float, z0: float = 0.0,
                yaw: float = 0.0, rgb=(0.6, 0.6, 0.6), kind: str = "structure") -> None:
        """Axis-aligned-in-its-own-frame box with its BASE at z0, centred at (cx, cy)."""
        h = np.array([sx / 2, sy / 2])
        corners = np.array([[-h[0], -h[1], 0], [h[0], -h[1], 0], [h[0], h[1], 0], [-h[0], h[1], 0],
                            [-h[0], -h[1], sz], [h[0], -h[1], sz], [h[0], h[1], sz], [-h[0], h[1], sz]])
        v = corners @ _yaw_matrix(yaw).T + np.array([cx, cy, z0])
        r, g, b = rgb
        sdf = (f'<model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {z0 + sz / 2:.4f} 0 0 {yaw:.5f}</pose>'
               f'<link name="link"><collision name="c"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry></collision>'
               f'<visual name="v"><geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
               f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
               f'<specular>0.1 0.1 0.1 1</specular></material></visual></link></model>')
        col = np.tile(np.array([int(r * 255), int(g * 255), int(b * 255)], np.uint8), (len(_BOX_FACES), 1))
        self.items.append(Item(name, sdf, v, _BOX_FACES, col, kind))


    def add_terrain(self, models_dir: Path, name: str, Z: np.ndarray, res: float, x0: float, y0: float,
                    rgb: np.ndarray) -> None:
        """Heightfield terrain as ONE mesh model (visual + collision) and map surface.

        Z[j, i] is the height at node (x0 + i*res, y0 + j*res). We deliberately
        build a triangle mesh instead of Gazebo's <heightmap>: a heightmap
        silently ignores link pose offsets (emap step 12 bug) and its collision
        grid is resampled, whereas the mesh we write IS the surface the map is
        computed from. `rgb` is an (H, W, 3) uint8 texture draped over x/y.
        """
        from PIL import Image

        ny, nx = Z.shape
        xs = x0 + res * np.arange(nx)
        ys = y0 + res * np.arange(ny)
        X, Y = np.meshgrid(xs, ys)
        verts = np.c_[X.ravel(), Y.ravel(), Z.ravel()]
        idx = np.arange(ny * nx).reshape(ny, nx)
        a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
        tris = np.concatenate([np.c_[a, b, c], np.c_[a, c, d]])         # counter-clockwise seen from above
        u = (verts[:, 0] - xs[0]) / (xs[-1] - xs[0])
        v = (verts[:, 1] - ys[0]) / (ys[-1] - ys[0])
        uv = np.c_[u, v][tris]                                           # (T,3,2)
        tex = models_dir / name / "terrain_texture.png"
        tex.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(np.flipud(rgb)).save(tex)                        # image row 0 = v 1
        export_mesh_model(models_dir, name, Mesh(name, verts, tris, uv), tex, f"{name} terrain mesh")
        h, w = rgb.shape[:2]
        tc = (verts[tris].mean(axis=1))
        ci = np.clip(((tc[:, 0] - xs[0]) / (xs[-1] - xs[0]) * w).astype(int), 0, w - 1)
        cj = np.clip(((tc[:, 1] - ys[0]) / (ys[-1] - ys[0]) * h).astype(int), 0, h - 1)
        cols = rgb[cj, ci]
        sdf = f'<include><name>{name}</name><uri>model://{name}</uri><pose>0 0 0 0 0 0</pose></include>'
        self.items.append(Item(name, sdf, verts, tris, cols, "ground"))

    # --------------------------------------------------------------- world --
    def world_sdf(self, extra_models: str = "", exclude_kinds: tuple[str, ...] = (),
                  see_through_kinds: tuple[str, ...] = ()) -> str:
        """World file text. `exclude_kinds` drops items; `see_through_kinds` keeps them (collision unchanged) but
        renders them 80 % transparent - used by the run videos so a deck does not hide the UGV from the overhead camera."""
        r, g, b = self.ground_rgb
        body = "\n    ".join(
            i.sdf.replace('<visual name="v">', '<visual name="v"><transparency>0.8</transparency>') if i.kind in see_through_kinds else i.sdf
            for i in self.items if i.kind not in exclude_kinds)
        return _WORLD_TEMPLATE.format(
            name=self.name, gsize=self.ground_size, gz=self.ground_z, r=r, g=g, b=b, body=body, extra=extra_models)

    # ----------------------------------------------------------- raster/map --
    def rasterize(self, res: float = 0.1, ceiling: float = np.inf, oversample: int = 4) -> Raster:
        x0, x1, y0, y1 = self.bounds
        R = Raster(x0, x1, y0, y1, res, oversample)
        for idx, it in enumerate(self.items):
            R.add(it.verts, it.tris, it.colours, idx, ceiling)
        R.finish()
        return R

    def ground_truth(self, res: float = 0.1, ceiling: float = 8.0, oversample: int = 4):
        """Return (layers dict, centre_x, centre_y, Raster) in nav-snapshot conventions."""
        from emap.traversability import compute_traversability

        R = self.rasterize(res, ceiling, oversample)
        valid = np.isfinite(R.height)
        elev = np.where(valid, R.height, self.ground_z).astype(np.float32)
        # Bare ground between items: the raster has no surface there, so treat
        # it as flat ground at `ground_z` (the ground plane in the world file).
        is_valid = np.ones_like(elev, dtype=np.float32)
        var = np.full_like(elev, CONFIDENT_VARIANCE)
        trav = compute_traversability(elev, var, is_valid.astype(bool), res, MAX_SLOPE, MAX_STEP, MAX_ROUGHNESS)
        layers = {"elevation": elev, "variance": var, "is_valid": is_valid, "traversability": trav.astype(np.float32)}
        x0, x1, y0, y1 = self.bounds
        # nav snapshots: centre of cell (row, col) = ((col - n/2)*res + cx, (row - n/2)*res + cy)
        cx = x0 + res / 2 + (R.nx / 2) * res
        cy = y0 + res / 2 + (R.ny / 2) * res
        return layers, cx, cy, R


_WORLD_TEMPLATE = """<?xml version="1.0"?>
<!-- GENERATED by src/nav/scripts/build_world.py - edit the world definition in
     src/nav/scripts/worldgen/worlds/, not this file. -->
<sdf version="1.6">
  <world name="{name}">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gui fullscreen="false">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.8 0.85 0.9</background_color>
        <camera_pose>-60 -60 45 0 0.5 0.8</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
    </gui>

    <!-- Directional light high above (z=500) like our other worlds: a light posed
         inside the sensing volume was once misread as geometry by the software
         depth camera (emap step 11). -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 500 0 0 0</pose>
      <diffuse>0.85 0.85 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 {gz} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{gsize} {gsize}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{gsize} {gsize}</size></plane></geometry>
          <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>
        </visual>
      </link>
    </model>

    {body}
    {extra}
  </world>
</sdf>
"""
