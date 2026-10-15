"""Minimal binary-FBX (v7100-7499) mesh reader, written from the public file layout.

Why this exists: the free CC0 construction-site pack we want to use ships as
FBX, Ignition Gazebo Fortress cannot load FBX, and this machine has no Blender
/ trimesh / assimp Python binding (and we do not install packages without
asking). FBX is a simple tree of "node records", so ~150 lines of numpy/zlib
is enough to pull out what we need: triangles, UVs and each mesh's world
transform. The output feeds (a) an OBJ exporter Gazebo can load and (b) the
ground-truth height-grid rasteriser - see `nav/worldgen/scene.py`.

File layout (version < 7500, 32-bit offsets):
  header : 21-byte magic "Kaydara FBX Binary  \\0", 2 bytes (0x1A 0x00), u32 version
  record : u32 end_offset, u32 n_props, u32 props_bytes, u8 name_len, name,
           properties..., nested records..., (13 zero bytes terminate a list)
  property types: Y i16, C bool, I i32, F f32, D f64, L i64 (scalars);
                  f d l i b = arrays: u32 count, u32 encoding (1 = zlib),
                  u32 compressed_bytes, data;  S string, R raw: u32 len, bytes.
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field

import numpy as np

_MAGIC = b"Kaydara FBX Binary  \x00\x1a\x00"
_ARRAY_DTYPES = {"f": "<f4", "d": "<f8", "l": "<i8", "i": "<i4", "b": "u1"}
_SCALARS = {"Y": "<h", "C": "?", "I": "<i", "F": "<f", "D": "<d", "L": "<q"}


@dataclass
class Node:
    name: str
    props: list = field(default_factory=list)
    children: list["Node"] = field(default_factory=list)

    def find(self, name: str) -> "Node | None":
        return next((c for c in self.children if c.name == name), None)

    def find_all(self, name: str) -> list["Node"]:
        return [c for c in self.children if c.name == name]


def _read_property(buf: bytes, pos: int):
    t = chr(buf[pos]); pos += 1
    if t in _SCALARS:
        fmt = _SCALARS[t]
        size = struct.calcsize(fmt)
        return struct.unpack_from(fmt, buf, pos)[0], pos + size
    if t in _ARRAY_DTYPES:
        count, enc, clen = struct.unpack_from("<III", buf, pos); pos += 12
        raw = buf[pos:pos + clen]; pos += clen
        if enc == 1:
            raw = zlib.decompress(raw)
        return np.frombuffer(raw, dtype=_ARRAY_DTYPES[t], count=count), pos
    if t in "SR":
        (n,) = struct.unpack_from("<I", buf, pos); pos += 4
        data = buf[pos:pos + n]; pos += n
        return (data.decode("utf-8", "replace") if t == "S" else data), pos
    raise ValueError(f"unknown FBX property type {t!r}")


def _read_node(buf: bytes, pos: int):
    end, n_props, props_len, name_len = struct.unpack_from("<IIIB", buf, pos)
    if end == 0:                       # 13-byte null record = end of a child list
        return None, pos + 13
    pos += 13
    name = buf[pos:pos + name_len].decode(); pos += name_len
    node = Node(name)
    for _ in range(n_props):
        p, pos = _read_property(buf, pos)
        node.props.append(p)
    while pos < end:                   # nested records fill the rest of the record
        child, pos = _read_node(buf, pos)
        if child is None:
            break
        node.children.append(child)
    return node, end


def parse_fbx(path: str) -> Node:
    """Parse a binary FBX file into a root Node whose children are the top-level records."""
    buf = open(path, "rb").read()
    if not buf.startswith(_MAGIC):
        raise ValueError(f"{path}: not a binary FBX (ASCII FBX unsupported)")
    pos = len(_MAGIC) + 4              # skip the version u32
    root = Node("__root__")
    while pos < len(buf):
        node, pos = _read_node(buf, pos)
        if node is None:
            break
        root.children.append(node)
    return root


# --------------------------------------------------------------------------- #
# Scene extraction: meshes + world transforms
# --------------------------------------------------------------------------- #
@dataclass
class Mesh:
    """One triangle mesh in WORLD axes (metres, Z up) after all FBX transforms."""
    name: str
    vertices: np.ndarray                 # (V, 3) float64
    triangles: np.ndarray                # (T, 3) int, indices into `vertices`
    uvs: np.ndarray | None = None        # (T, 3, 2) per-triangle-corner UVs
    material: str = ""


def _p70(node: Node) -> dict:
    """Properties70 of a Model as {name: tuple of values}."""
    out = {}
    pr = node.find("Properties70")
    if pr is not None:
        for p in pr.children:
            if p.name == "P":
                out[p.props[0]] = tuple(p.props[4:])
    return out


def _rot(euler_deg, order_xyz=True) -> np.ndarray:
    """Rotation matrix for FBX's default Euler order (XYZ => R = Rz @ Ry @ Rx)."""
    rx, ry, rz = np.radians(euler_deg)
    cx, sx, cy, sy, cz, sz = np.cos(rx), np.sin(rx), np.cos(ry), np.sin(ry), np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _local_matrix(p: dict) -> np.ndarray:
    """T * R_pre * R * R_post^-1 * S  (pivots/offsets are zero in this pack)."""
    T = np.array(p.get("Lcl Translation", (0, 0, 0)), float)
    S = np.array(p.get("Lcl Scaling", (1, 1, 1)), float)
    R = _rot(p.get("Lcl Rotation", (0, 0, 0)))
    Rpre = _rot(p.get("PreRotation", (0, 0, 0)))
    Rpost = _rot(p.get("PostRotation", (0, 0, 0)))
    M = np.eye(4)
    M[:3, :3] = Rpre @ R @ np.linalg.inv(Rpost) @ np.diag(S)
    M[:3, 3] = T
    return M


def _triangulate(poly_idx: np.ndarray):
    """FBX polygons: a negative index (~i) marks the last corner. Fan-triangulate.

    Returns (tri_vertex_indices (T,3), corner_indices (T,3)) where corner indices
    address the per-polygon-corner arrays (UVs with ByPolygonVertex mapping).
    """
    tris, corners, start = [], [], 0
    for i, v in enumerate(poly_idx):
        if v < 0:
            ids = list(range(start, i + 1))
            vs = [int(poly_idx[j]) if poly_idx[j] >= 0 else ~int(poly_idx[j]) for j in ids]
            for k in range(1, len(ids) - 1):
                tris.append((vs[0], vs[k], vs[k + 1]))
                corners.append((ids[0], ids[k], ids[k + 1]))
            start = i + 1
    return np.array(tris, int).reshape(-1, 3), np.array(corners, int).reshape(-1, 3)


def load_meshes(path: str, scale: float | None = None) -> list[Mesh]:
    """Load every mesh in an FBX as world-space (Z up, metres) triangle meshes.

    `scale` multiplies all coordinates; default derives metres from the file's
    UnitScaleFactor (cm = 1.0 in FBX terms, so factor/100 gives metres).
    """
    root = parse_fbx(path)
    gs = {k: v for k, v in _p70(root.find("GlobalSettings")).items()}
    up = int(gs.get("UpAxis", (1,))[0])
    unit = float(gs.get("UnitScaleFactor", (1.0,))[0])
    s = scale if scale is not None else unit / 100.0

    objs = root.find("Objects")
    geoms = {int(n.props[0]): n for n in objs.find_all("Geometry")}
    models = {int(n.props[0]): n for n in objs.find_all("Model")}
    mats = {int(n.props[0]): n.props[1].split("\x00")[0] for n in objs.find_all("Material")}
    parent_of, geom_of, mat_of = {}, {}, {}
    for c in root.find("Connections").find_all("C"):
        kind, child, parent = c.props[0], int(c.props[1]), int(c.props[2])
        if child in models and parent in models:
            parent_of[child] = parent
        elif child in geoms and parent in models:
            geom_of[parent] = child
        elif child in mats and parent in models:
            mat_of[parent] = mats[child]

    # Y-up files -> Z-up world: rotate +90 deg about X (y -> z, z -> -y).
    axis_fix = np.eye(4)
    if up == 1:
        axis_fix[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

    def world_matrix(mid):
        M, cur = np.eye(4), mid
        while cur in models:                         # compose parent chain, root first
            M = _local_matrix(_p70(models[cur])) @ M
            cur = parent_of.get(cur)
        return M

    meshes = []
    for mid, gid in geom_of.items():
        g = geoms[gid]
        verts = np.asarray(g.find("Vertices").props[0], float).reshape(-1, 3)
        pidx = np.asarray(g.find("PolygonVertexIndex").props[0])
        tris, corners = _triangulate(pidx)
        M = axis_fix @ world_matrix(mid)
        v = (np.c_[verts, np.ones(len(verts))] @ M.T)[:, :3] * s
        uvs = None
        le = g.find("LayerElementUV")
        if le is not None and le.find("UV") is not None:
            uv = np.asarray(le.find("UV").props[0], float).reshape(-1, 2)
            ui = le.find("UVIndex")
            mapping = le.find("MappingInformationType").props[0]
            if mapping == "ByPolygonVertex" and ui is not None:
                idx = np.asarray(ui.props[0])[corners]
                uvs = uv[idx]
            elif mapping == "ByPolygonVertex":
                uvs = uv[corners]
            elif mapping == "ByVertice" or mapping == "ByVertex":
                uvs = uv[tris]
        name = models[mid].props[1].split("\x00")[0]
        meshes.append(Mesh(name, v, tris, uvs, mat_of.get(mid, "")))
    return meshes
