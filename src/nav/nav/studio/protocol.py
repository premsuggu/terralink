"""Wire format between the studio server and the browser.

TWO KINDS OF WEBSOCKET FRAMES
-----------------------------
* TEXT frames carry JSON: small, human-readable messages (`hello`, `state`, `error`, ...). They are easy to debug in a browser console.
* BINARY frames carry the elevation map. A 560 x 560 map with 6 float32 layers is 7.5 MB, far too much to send twice a second as JSON. The browser
  only needs three things per cell (height, "was it seen", "can the UGV drive there"), so we send:

      byte 0         frame type (0x01 = elevation map)
      bytes 1..4     uint32 little-endian: length H of the JSON header
      next H bytes   JSON header (see below)
      next h*w*2     elevation as int16 little-endian, millimetres; -32768 means "not observed"
      next h*w       flags as uint8: bit 0 = observed, bit 1 = walkable (what the planner will use)

  and only the part of the grid that has been observed (its bounding box plus a small margin), so the first frames of a flight are tiny.
  A full W2 map is about 0.9 MB; localhost handles that at 1 Hz easily.

MAP GEOMETRY (the same convention as `nav.replay.MapSnapshot` and the planner)
-----------------------------------------------------------------------------
The full grid has `rows` x `cols` cells of `res` metres; cell (row, col) has its centre at
    x = (col + 0.5 - cols / 2) * res + cx        y = (row + 0.5 - rows / 2) * res + cy
where (cx, cy) is the map centre in the map frame. The cropped block that is sent starts at full-grid cell (r0, c0) and is h x w cells.

Example (a client in Python)::

    pm = unpack_map(frame)            # frame = bytes received over the websocket
    full_elevation = pm.full()["elevation"]   # (rows, cols) float32, NaN where not observed
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass

import numpy as np

FRAME_MAP = 0x01
PROTOCOL_VERSION = 1

# int16 value that means "this cell was never observed" (elevation in mm never reaches it: heights are clipped to +-32.767 m)
_NO_DATA = -32768
FLAG_VALID = 0x01
FLAG_WALKABLE = 0x02


def pack_map(
    elevation: np.ndarray,
    is_valid: np.ndarray,
    walkable: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    version: int,
    margin: int = 3,
) -> bytes:
    """Pack one map into a binary frame (layout in the module docstring).

    `version` is a counter the caller increments per published map; the browser uses it to ignore frames that arrive out of order.
    `margin` extra cells are kept around the observed area so the viewer can draw a soft edge.
    """
    valid = np.asarray(is_valid).astype(bool)
    walk = np.asarray(walkable).astype(bool) & valid          # a cell nobody has seen can never be walkable
    rows, cols = valid.shape
    if valid.any():
        # bounding box of everything observed: the first and last row/column that contain at least one observed cell
        rr = np.flatnonzero(valid.any(axis=1))
        cc = np.flatnonzero(valid.any(axis=0))
        r0, r1 = max(int(rr[0]) - margin, 0), min(int(rr[-1]) + 1 + margin, rows)
        c0, c1 = max(int(cc[0]) - margin, 0), min(int(cc[-1]) + 1 + margin, cols)
    else:
        r0 = c0 = r1 = c1 = 0                                  # nothing observed yet: an empty block
    h, w = r1 - r0, c1 - c0
    v = valid[r0:r1, c0:c1]
    elev_mm = np.rint(np.nan_to_num(np.asarray(elevation, dtype=np.float64)[r0:r1, c0:c1], nan=0.0) * 1000.0)
    elev_i16 = np.where(v, np.clip(elev_mm, -32767, 32767), _NO_DATA).astype("<i2")
    flags = (v.astype(np.uint8) * FLAG_VALID) | (walk[r0:r1, c0:c1].astype(np.uint8) * FLAG_WALKABLE)
    header = json.dumps({
        "v": int(version), "res": float(resolution), "cx": float(center_x), "cy": float(center_y),
        "rows": int(rows), "cols": int(cols), "r0": int(r0), "c0": int(c0), "h": int(h), "w": int(w),
    }).encode("utf-8")
    return bytes([FRAME_MAP]) + struct.pack("<I", len(header)) + header + elev_i16.tobytes() + flags.tobytes()


@dataclass
class PackedMap:
    """A decoded map frame (used by tests and by Python clients; the browser has its own JavaScript decoder)."""

    header: dict
    elevation_m: np.ndarray   # (h, w) float32, NaN where not observed
    valid: np.ndarray         # (h, w) bool
    walkable: np.ndarray      # (h, w) bool

    def full(self) -> dict[str, np.ndarray]:
        """The three layers expanded back to the full (rows, cols) grid."""
        hd = self.header
        shape = (hd["rows"], hd["cols"])
        elev = np.full(shape, np.nan, dtype=np.float32)
        valid = np.zeros(shape, dtype=bool)
        walk = np.zeros(shape, dtype=bool)
        r0, c0, h, w = hd["r0"], hd["c0"], hd["h"], hd["w"]
        elev[r0:r0 + h, c0:c0 + w] = self.elevation_m
        valid[r0:r0 + h, c0:c0 + w] = self.valid
        walk[r0:r0 + h, c0:c0 + w] = self.walkable
        return {"elevation": elev, "valid": valid, "walkable": walk}


def unpack_map(frame: bytes) -> PackedMap:
    """Decode a binary frame made by `pack_map`. Raises ValueError for anything malformed."""
    if len(frame) < 5 or frame[0] != FRAME_MAP:
        raise ValueError("not a map frame")
    (hlen,) = struct.unpack_from("<I", frame, 1)
    if 5 + hlen > len(frame):
        raise ValueError("truncated header")
    header = json.loads(frame[5:5 + hlen].decode("utf-8"))
    h, w = header["h"], header["w"]
    n = h * w
    body = frame[5 + hlen:]
    if len(body) != n * 3:
        raise ValueError(f"body is {len(body)} bytes, expected {n * 3}")
    elev_i16 = np.frombuffer(body[: n * 2], dtype="<i2").reshape(h, w)
    flags = np.frombuffer(body[n * 2:], dtype=np.uint8).reshape(h, w)
    valid = (flags & FLAG_VALID) != 0
    elev = np.where(elev_i16 == _NO_DATA, np.nan, elev_i16.astype(np.float32) / 1000.0).astype(np.float32)
    return PackedMap(header=header, elevation_m=elev, valid=valid, walkable=(flags & FLAG_WALKABLE) != 0)


def dumps(message: dict) -> str:
    """JSON for a text frame: compact, and NaN/inf become null (browsers' JSON.parse rejects the literal NaN)."""

    def clean(x):
        if isinstance(x, float) and not np.isfinite(x):
            return None
        if isinstance(x, dict):
            return {k: clean(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [clean(v) for v in x]
        if isinstance(x, np.generic):
            return clean(x.item())
        return x

    return json.dumps(clean(message), separators=(",", ":"))
