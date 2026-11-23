// Decoder for the binary map frame sent by the studio server (the layout is documented in nav/studio/protocol.py):
//   byte 0 = 0x01, bytes 1..4 = uint32 LE header length H, then H bytes of JSON, then h*w int16 (millimetres; -32768 = never observed),
//   then h*w flag bytes (bit0 = observed, bit1 = walkable i.e. what the planner will treat as drivable).
// Pure functions only - no DOM, no three.js - so the logic is easy to reason about.

export const NO_DATA = -32768;

export function decodeMap(buf) {
  const dv = new DataView(buf);
  if (buf.byteLength < 5 || dv.getUint8(0) !== 1) throw new Error('not a map frame');
  const hlen = dv.getUint32(1, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 5, hlen)));
  const n = header.h * header.w;
  const off = 5 + hlen;
  if (buf.byteLength !== off + n * 3) throw new Error(`map body is ${buf.byteLength - off} bytes, expected ${n * 3}`);
  // `off` can be odd, and an Int16Array view needs an even byte offset, so copy those bytes out (slice() returns a fresh aligned buffer).
  const elevMm = new Int16Array(buf.slice(off, off + n * 2));
  const flags = new Uint8Array(buf, off + n * 2, n);
  return { header, elevMm, flags };
}

// A decoded map with the geometry helpers the viewer needs (world <-> cell, height lookup, ray picking).
export class MapGrid {
  constructor({ header, elevMm, flags }) {
    this.version = header.v;
    this.res = header.res;
    this.h = header.h;
    this.w = header.w;
    this.rows = header.rows;
    this.cols = header.cols;
    this.cx = header.cx;
    this.cy = header.cy;
    this.elevMm = elevMm;
    this.flags = flags;
    // Cell (row, col) of the FULL grid is centred at x = (col + 0.5 - cols/2)*res + cx (and likewise y). The block we hold starts at (r0, c0).
    this.x0 = (header.c0 + 0.5 - header.cols / 2) * header.res + header.cx;
    this.y0 = (header.r0 + 0.5 - header.rows / 2) * header.res + header.cy;
    // Extent of the whole map grid (not just the observed block) - drawn as the world boundary.
    this.bounds = {
      xmin: header.cx - (header.cols * header.res) / 2, xmax: header.cx + (header.cols * header.res) / 2,
      ymin: header.cy - (header.rows * header.res) / 2, ymax: header.cy + (header.rows * header.res) / 2,
    };
  }

  isObserved(i) { return (this.flags[i] & 1) !== 0; }
  isWalkable(i) { return (this.flags[i] & 2) !== 0; }
  height(i) { return this.elevMm[i] / 1000; }

  // Index into the block of the cell containing world point (x, y), or -1 if outside the block.
  cellIndex(x, y) {
    const c = Math.round((x - this.x0) / this.res);
    const r = Math.round((y - this.y0) / this.res);
    return (c >= 0 && c < this.w && r >= 0 && r < this.h) ? r * this.w + c : -1;
  }

  // Terrain height at (x, y), or NaN where nothing has been observed.
  heightAt(x, y) {
    const i = this.cellIndex(x, y);
    return (i >= 0 && this.isObserved(i)) ? this.height(i) : NaN;
  }

  // Find where a ray first meets the terrain. The ray is marched in half-cell steps; where no cell was observed the ground plane z = floorZ
  // answers instead, flagged observed:false. Returns {x, y, z, observed, walkable} or null (ray never comes down within maxDist).
  raycast(ox, oy, oz, dx, dy, dz, maxDist = 600, floorZ = 0) {
    const step = this.res * 0.5;
    const hitAt = (t) => {
      const x = ox + dx * t, y = oy + dy * t, z = oz + dz * t;
      const i = this.cellIndex(x, y);
      if (i >= 0 && this.isObserved(i)) return z <= this.height(i) ? { x, y, z: this.height(i), observed: true, walkable: this.isWalkable(i) } : null;
      return z <= floorZ ? { x, y, z: floorZ, observed: false, walkable: false } : null;
    };
    let prev = 0;
    for (let t = 0; t <= maxDist; t += step) {
      if (hitAt(t)) {
        // refine between the last miss and this hit (bisection) so the picked point is not off by half a cell
        let lo = prev, hi = t;
        for (let k = 0; k < 8; k++) { const mid = 0.5 * (lo + hi); if (hitAt(mid)) hi = mid; else lo = mid; }
        return hitAt(hi);
      }
      prev = t;
    }
    return null;
  }
}
