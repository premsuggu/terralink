// The "real site": the solid things of the simulated world (containers, fences, walls) drawn as boxes.
// The server sends them once in the `hello` message (nav/studio/scenery.py). They are for the HUMAN: the robots never see this layer, they
// only know what they have scanned. So it is drawn in muted colours with dark outlines, and the scanned map (brighter) is drawn over it.
// THREE is the global from vendor/three.min.js. Z is up.

export class Scenery {
  constructor(scene) {
    this.group = new THREE.Group();
    scene.add(this.group);
    this.bounds = null;                   // {xmin, xmax, ymin, ymax} of everything drawn, or null
    this._geo = new THREE.BoxGeometry(1, 1, 1);
    this._edges = new THREE.EdgesGeometry(this._geo);
    this._edgeMat = new THREE.LineBasicMaterial({ color: 0x0b0f14, transparent: true, opacity: 0.55 });
  }

  // items: [{x, y, z, roll, pitch, yaw, sx, sy, sz, rgb:[r,g,b]}, ...]
  set(items) {
    for (const o of [...this.group.children]) { this.group.remove(o); o.material?.dispose?.(); }
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    for (const it of items || []) {
      // slightly desaturated + darkened so the real site reads as "background" next to the brighter scanned map
      const col = new THREE.Color(it.rgb[0], it.rgb[1], it.rgb[2]).multiplyScalar(0.82);
      const mesh = new THREE.Mesh(this._geo, new THREE.MeshStandardMaterial({ color: col, roughness: 0.9, metalness: 0.0 }));
      mesh.scale.set(it.sx, it.sy, it.sz);
      mesh.position.set(it.x, it.y, it.z);
      mesh.rotation.set(it.roll || 0, it.pitch || 0, it.yaw || 0, 'ZYX');    // SDF poses are roll-pitch-yaw about the fixed axes = intrinsic ZYX
      mesh.add(new THREE.LineSegments(this._edges, this._edgeMat));          // outline: the box's own unit-cube edges scale with it
      this.group.add(mesh);
      const r = 0.5 * (Math.abs(it.sx * Math.cos(it.yaw || 0)) + Math.abs(it.sy * Math.sin(it.yaw || 0)));
      const r2 = 0.5 * (Math.abs(it.sx * Math.sin(it.yaw || 0)) + Math.abs(it.sy * Math.cos(it.yaw || 0)));
      xmin = Math.min(xmin, it.x - r); xmax = Math.max(xmax, it.x + r); ymin = Math.min(ymin, it.y - r2); ymax = Math.max(ymax, it.y + r2);
    }
    this.bounds = isFinite(xmin) ? { xmin, xmax, ymin, ymax } : null;
  }

  set visible(v) { this.group.visible = v; }
  get count() { return this.group.children.length; }
}
