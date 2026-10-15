"""Top-down rasteriser for triangle meshes: height grid + colour image.

One z-buffer pass over every triangle gives BOTH products we need:
  * `height` - the highest surface z in each cell. This is exactly what an
               ideal overhead depth camera would measure, i.e. the
               ground-truth 2.5D elevation map we hand to the planner instead
               of flying the UAV.
  * `colour` - colour of that top-most surface, for review images.

CONSERVATIVE coverage (important - this grid is what A* is judged against):
  1. Super-sampling. The grid is first filled at `oversample` x the target
     resolution, then max-pooled down. A cell therefore counts as occupied
     if ANY part of a triangle overlaps it, not only if the triangle covers
     the cell centre - a 0.15 m wall or a fence post can no longer slip
     between cell centres (the "thin wall seam" failure of step 08, but
     inside the ground truth).
  2. Steep / vertical faces. A fence or wall modelled as vertical quads has
     zero plan-view area, so area filling never touches it. For every steep
     triangle we additionally walk its three plan-view edges with a dense
     line and write the interpolated z (the "supercover" of the footprint).
  3. `ceiling`: triangles whose LOWEST point is at or above this height are
     ignored (an overhead crane jib or an upper storey a ground robot can
     drive under). A tall wall that starts at the ground is kept at its full
     height - only things that hover are dropped.
"""
from __future__ import annotations

import numpy as np

# A triangle is "steep" when |normal_z| is below this (vertical = 0, flat = 1).
_STEEP_NZ = 0.35


class Raster:
    def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float, res: float, oversample: int = 4):
        self.res, self.k = res, int(oversample)
        self.x_min, self.y_min = x_min, y_min
        self.nx = int(round((x_max - x_min) / res))
        self.ny = int(round((y_max - y_min) / res))
        self.fres = res / self.k                         # fine-grid cell size
        # Row index = y, column index = x (same convention as the nav snapshots).
        self._h = np.full((self.ny * self.k, self.nx * self.k), -np.inf, np.float32)
        self._c = np.zeros((self.ny * self.k, self.nx * self.k, 3), np.uint8)
        self._i = np.full((self.ny * self.k, self.nx * self.k), -1, np.int16)

    # ----------------------------------------------------------------- fill --
    def _put(self, rr, cc, zz, col, item_id) -> None:
        """Z-buffered write of fine cells (rr, cc) with heights zz."""
        ok = (rr >= 0) & (rr < self._h.shape[0]) & (cc >= 0) & (cc < self._h.shape[1])
        rr, cc, zz = rr[ok], cc[ok], zz[ok]
        better = zz > self._h[rr, cc]
        rr, cc, zz = rr[better], cc[better], zz[better]
        self._h[rr, cc] = zz
        self._c[rr, cc] = col
        self._i[rr, cc] = item_id

    def add(self, verts: np.ndarray, tris: np.ndarray, colours: np.ndarray, item_id: int = 0,
            ceiling: float = np.inf) -> None:
        """verts (V,3) world metres; tris (T,3); colours (T,3) uint8 per triangle."""
        P = verts[tris].astype(np.float64)                      # (T,3,3)
        keep = P[..., 2].min(axis=1) < ceiling
        P, colours = P[keep], colours[keep]
        f = self.fres
        fx = (P[..., 0] - self.x_min) / f                       # fine-grid fractional coords
        fy = (P[..., 1] - self.y_min) / f
        n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
        nlen = np.linalg.norm(n, axis=1)
        nz = np.abs(n[:, 2]) / np.maximum(nlen, 1e-12)
        for t in range(len(P)):
            col = colours[t]
            # --- (1) area fill by fine cell centres -------------------------
            x0, x1, y0, y1 = fx[t].min(), fx[t].max(), fy[t].min(), fy[t].max()
            c0, c1 = int(np.floor(x0 - 0.5)), int(np.ceil(x1 - 0.5))
            r0, r1 = int(np.floor(y0 - 0.5)), int(np.ceil(y1 - 0.5))
            if (c1 - c0 + 1) * (r1 - r0 + 1) <= 4_000_000 and nz[t] > 1e-6:
                cs, rs = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
                qx, qy = cs + 0.5, rs + 0.5
                ax, ay, bx, by, cx, cy = fx[t, 0], fy[t, 0], fx[t, 1], fy[t, 1], fx[t, 2], fy[t, 2]
                den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
                if abs(den) > 1e-12:
                    w0 = ((by - cy) * (qx - cx) + (cx - bx) * (qy - cy)) / den
                    w1 = ((cy - ay) * (qx - cx) + (ax - cx) * (qy - cy)) / den
                    w2 = 1.0 - w0 - w1
                    inside = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9)
                    if inside.any():
                        z = w0 * P[t, 0, 2] + w1 * P[t, 1, 2] + w2 * P[t, 2, 2]
                        self._put(rs[inside], cs[inside], z[inside], col, item_id)
            # --- (2) edge supercover for steep / zero-area faces -------------
            if nz[t] < _STEEP_NZ:
                for a, b in ((0, 1), (1, 2), (2, 0)):
                    length = np.hypot(fx[t, b] - fx[t, a], fy[t, b] - fy[t, a])
                    m = int(np.ceil(length * 2)) + 1            # 2 samples per fine cell
                    s = np.linspace(0.0, 1.0, m)
                    ex = fx[t, a] + s * (fx[t, b] - fx[t, a])
                    ey = fy[t, a] + s * (fy[t, b] - fy[t, a])
                    ez = P[t, a, 2] + s * (P[t, b, 2] - P[t, a, 2])
                    self._put(np.floor(ey).astype(int), np.floor(ex).astype(int), ez, col, item_id)

    # --------------------------------------------------------------- results --
    def finish(self) -> None:
        """Max-pool the fine buffers down to the target resolution."""
        k = self.k
        h = self._h.reshape(self.ny, k, self.nx, k).transpose(0, 2, 1, 3).reshape(self.ny, self.nx, k * k)
        arg = h.argmax(axis=2)
        self.height = np.take_along_axis(h, arg[..., None], 2)[..., 0].astype(np.float64)
        c = self._c.reshape(self.ny, k, self.nx, k, 3).transpose(0, 2, 1, 3, 4).reshape(self.ny, self.nx, k * k, 3)
        self.colour = np.take_along_axis(c, arg[..., None, None], 2)[:, :, 0, :]
        i = self._i.reshape(self.ny, k, self.nx, k).transpose(0, 2, 1, 3).reshape(self.ny, self.nx, k * k)
        self.item = np.take_along_axis(i, arg[..., None], 2)[..., 0]
        del self._h, self._c, self._i
