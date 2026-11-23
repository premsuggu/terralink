// The ground: a floor plane (shows where nothing has been scanned yet) plus a height-field mesh built from the newest map frame.
// THREE is the global from vendor/three.min.js. The world is Z-up (x east, y north, z up) like the ROS map frame.

// flat ground = calm green, rising through yellow to orange-red for tall things (so a scanned yard reads as ground + obstacles)
const HEIGHT_STOPS = [[0.0, [0.30, 0.62, 0.38]], [0.2, [0.45, 0.70, 0.36]], [0.5, [0.85, 0.80, 0.32]], [0.8, [0.90, 0.55, 0.25]], [1.0, [0.78, 0.30, 0.22]]];

// Smooth colour for t in [0, 1] (low = blue, high = red-brown).
export function heightColour(t, out) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < HEIGHT_STOPS.length; i++) {
    if (t <= HEIGHT_STOPS[i][0]) {
      const [t0, c0] = HEIGHT_STOPS[i - 1], [t1, c1] = HEIGHT_STOPS[i];
      const f = (t - t0) / (t1 - t0);
      out[0] = c0[0] + (c1[0] - c0[0]) * f; out[1] = c0[1] + (c1[1] - c0[1]) * f; out[2] = c0[2] + (c1[2] - c0[2]) * f;
      return out;
    }
  }
  return out;
}

export class Terrain {
  constructor(scene) {
    this.scene = scene;
    this.mesh = null;
    this.grid = null;
    this.mode = 'height';          // 'height' | 'drivable' | 'plain'
    this.reach = null;
    this.showScanned = true;       // the page's "show the scanned map" switch
    this.heightRange = [0, 1];
    this._makeFloor();
    this.outline = null;
  }

  // MuJoCo-style checkered floor under everything: wherever the map has no data you see this instead of terrain.
  _makeFloor() {
    const c = document.createElement('canvas');
    c.width = c.height = 64;
    const g = c.getContext('2d');
    g.fillStyle = '#3a4656'; g.fillRect(0, 0, 64, 64);
    g.fillStyle = '#425064'; g.fillRect(0, 0, 32, 32); g.fillRect(32, 32, 32, 32);
    const tex = new THREE.CanvasTexture(c);
    tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
    tex.repeat.set(100, 100);          // one texture tile = 2 m, so each checker square is 1 m
    this.floor = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), new THREE.MeshBasicMaterial({ map: tex }));
    this.floor.position.z = -0.03;      // just under z = 0 so flat terrain does not flicker against it
    this.scene.add(this.floor);
  }

  // Replace the height field with a new map frame (a MapGrid).
  update(grid) {
    this.grid = grid;
    this.floor.position.x = grid.cx; this.floor.position.y = grid.cy;
    this._drawOutline(grid.bounds);
    const { h, w, res } = grid;
    if (h < 2 || w < 2) { if (this.mesh) this.mesh.visible = false; return; }     // nothing observed yet: just the floor
    if (this.mesh) this.mesh.visible = this.showScanned;
    const n = h * w;
    const pos = new Float32Array(n * 3);
    const idx = new Uint32Array(Math.max(0, (h - 1) * (w - 1) * 6));
    let ni = 0;
    for (let r = 0; r < h; r++) {
      for (let c = 0; c < w; c++) {
        const i = r * w + c;
        pos[i * 3] = grid.x0 + c * res; pos[i * 3 + 1] = grid.y0 + r * res; pos[i * 3 + 2] = grid.isObserved(i) ? grid.height(i) : 0;
      }
    }
    // A quad is drawn only if all four of its corner cells were observed, so unscanned ground stays visibly empty.
    for (let r = 0; r < h - 1; r++) {
      for (let c = 0; c < w - 1; c++) {
        const i00 = r * w + c, i01 = i00 + 1, i10 = i00 + w, i11 = i10 + 1;
        if (!(grid.isObserved(i00) && grid.isObserved(i01) && grid.isObserved(i10) && grid.isObserved(i11))) continue;
        idx[ni++] = i00; idx[ni++] = i01; idx[ni++] = i11;       // counter-clockwise seen from above (+x then +y), so the face points up
        idx[ni++] = i00; idx[ni++] = i11; idx[ni++] = i10;
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
    geo.setIndex(new THREE.BufferAttribute(idx.slice(0, ni), 1));
    if (!this.mesh) {
      // flatShading: the lighting normal comes from the screen-space derivative of each face, so no per-vertex normals are needed.
      this.mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ vertexColors: true, flatShading: true, roughness: 0.95, metalness: 0, side: THREE.DoubleSide,
        // the scanned map wins where it lies exactly on top of the real-site boxes (a scanned container roof), instead of flickering with it
        polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 }));
      this.mesh.frustumCulled = false;
      this.mesh.visible = this.showScanned;
      this.scene.add(this.mesh);
    } else {
      this.mesh.geometry.dispose();
      this.mesh.geometry = geo;
    }
    this._recolor();
  }

  // `reach`: Uint8Array over the map block, 1 = connected to the UGV over scanned drivable cells (see app.js computeReach), or null
  setReach(reach) { this.reach = reach; if (this.mesh && this.mode === 'drivable') this._recolor(); }

  setScannedVisible(v) { this.showScanned = v; if (this.mesh) this.mesh.visible = v && this.grid && this.grid.h >= 2; }

  setMode(mode) {
    this.mode = mode;
    if (this.grid) this._recolor();
  }

  _recolor() {
    const g = this.grid, col = this.mesh.geometry.getAttribute('color'), n = g.h * g.w;
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < n; i++) if (g.isObserved(i)) { const z = g.height(i); if (z < lo) lo = z; if (z > hi) hi = z; }
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    hi = Math.max(hi, lo + 0.5);
    this.heightRange = [lo, hi];
    const tmp = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      let c;
      // drivable: green = the UGV can get there over scanned ground, steel blue = drivable ground that is cut off from the UGV (a container roof, an
      // island between obstacles), red = blocked, dark = not scanned
      if (this.mode === 'drivable') c = !g.isObserved(i) ? [0.2, 0.2, 0.2] : g.isWalkable(i) ? (this.reach && !this.reach[i] ? [0.36, 0.52, 0.66] : [0.23, 0.66, 0.42]) : [0.85, 0.34, 0.25];
      else if (this.mode === 'plain') c = [0.72, 0.70, 0.65];
      else c = g.isObserved(i) ? heightColour((g.height(i) - lo) / (hi - lo), tmp) : [0.2, 0.2, 0.2];
      col.array[i * 3] = c[0]; col.array[i * 3 + 1] = c[1]; col.array[i * 3 + 2] = c[2];
    }
    col.needsUpdate = true;
  }

  // Thin rectangle showing how far the map grid extends (the planner cannot plan outside it).
  _drawOutline(b) {
    if (this.outline) { this.scene.remove(this.outline); this.outline.geometry.dispose(); }
    const pts = [b.xmin, b.ymin, 0.02, b.xmax, b.ymin, 0.02, b.xmax, b.ymax, 0.02, b.xmin, b.ymax, 0.02];
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pts), 3));
    this.outline = new THREE.LineLoop(geo, new THREE.LineBasicMaterial({ color: 0x5b7fa8, transparent: true, opacity: 0.5 }));
    this.scene.add(this.outline);
  }

  // Terrain height under a point, or `fallback` where nothing is known (used to stand vehicles and draw lines on the ground).
  heightAt(x, y, fallback = 0) {
    const z = this.grid ? this.grid.heightAt(x, y) : NaN;
    return Number.isNaN(z) ? fallback : z;
  }
}
