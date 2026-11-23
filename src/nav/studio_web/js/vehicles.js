// Simple models of the two robots plus the things drawn around them (beacon, trails, goal, plan, camera footprint).
// Real dimensions: the UAV is about 0.55 m across and the UGV (Jackal class) about 0.5 x 0.43 m - tiny on a 60 m map - so everything is
// drawn at `scale` times real size (a slider in the page) and each vehicle carries a "beacon" cone that stays a readable size on screen.

const mat = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.7, metalness: 0.1, ...extra });

export function makeUav() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.22, 0.08), mat(0x2b2f36));
  g.add(body);
  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.12, 0.06), mat(0xff7a1a));   // orange = the front (+x), so you can see which way it faces
  nose.position.x = 0.19; g.add(nose);
  g.userData.rotors = [];
  for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {      // X-shaped frame
    const arm = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.03, 0.03), mat(0x444a55));
    arm.position.set(sx * 0.14, sy * 0.14, 0); arm.rotation.z = sx * sy * Math.PI / 4; g.add(arm);
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.15, 0.15, 0.01, 20), new THREE.MeshBasicMaterial({ color: 0x9fb4cc, transparent: true, opacity: 0.35 }));
    rotor.rotation.x = Math.PI / 2; rotor.position.set(sx * 0.27, sy * 0.27, 0.05); g.add(rotor);
    g.userData.rotors.push(rotor);
  }
  return g;
}

export function makeUgv() {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.3, 0.16), mat(0xf2c200));       // Jackal yellow
  body.position.z = 0.17; g.add(body);
  const deck = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.26, 0.04), mat(0x20242a));
  deck.position.z = 0.27; g.add(deck);
  const lidar = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.06, 14), mat(0x111111));
  lidar.rotation.x = Math.PI / 2; lidar.position.set(0.1, 0, 0.32); g.add(lidar);
  const nose = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.2, 0.05), mat(0xd9381e));       // red = the front (+x)
  nose.position.set(0.27, 0, 0.17); g.add(nose);
  for (const sx of [0.17, -0.17]) for (const sy of [0.22, -0.22]) {
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.098, 0.098, 0.1, 18), mat(0x15181c));
    wheel.position.set(sx, sy, 0.098); g.add(wheel);                                       // cylinder axis is Y by default = the axle direction
  }
  return g;
}

// A downward cone above a vehicle. Its size is set per camera (`sizeFor`) so it stays about the same size on screen.
export function makeBeacon(color) {
  const cone = new THREE.Mesh(new THREE.ConeGeometry(0.5, 1.4, 12), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9, depthTest: false }));
  cone.renderOrder = 10;
  const holder = new THREE.Group();   // the holder's origin is the cone's TIP, so placing it at a vehicle puts the tip on the vehicle
  cone.rotation.x = -Math.PI / 2;     // a cone's tip points along +Y; this turns it to point down (-Z)
  cone.position.z = 0.7;              // half the cone's height: the tip then sits at the holder's origin
  holder.add(cone);
  return holder;
}

// A growing line (the path a vehicle has travelled), stored in one preallocated buffer.
export class Trail {
  constructor(scene, color, maxPoints = 1500) {
    this.max = maxPoints;
    this.n = 0;
    this.pos = new Float32Array(maxPoints * 3);
    this.geo = new THREE.BufferGeometry();
    this.geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3));
    this.geo.setDrawRange(0, 0);
    this.line = new THREE.Line(this.geo, new THREE.LineBasicMaterial({ color }));
    this.line.frustumCulled = false;
    scene.add(this.line);
    this.last = null;
  }
  // Add a point if it is at least `minStep` metres from the previous one (otherwise a hovering UAV would fill the buffer with duplicates).
  add(x, y, z, minStep = 0.15) {
    if (this.last && Math.hypot(x - this.last[0], y - this.last[1], z - this.last[2]) < minStep) return;
    if (this.n >= this.max) { this.pos.copyWithin(0, 3 * (this.max / 2), 3 * this.max); this.n = this.max / 2; }   // full: drop the older half
    this.pos.set([x, y, z], this.n * 3); this.n++;
    this.last = [x, y, z];
    this.geo.attributes.position.needsUpdate = true;
    this.geo.setDrawRange(0, this.n);
  }
  clear() { this.n = 0; this.last = null; this.geo.setDrawRange(0, 0); }
  set visible(v) { this.line.visible = v; }
}

// The goal flag: a pole with a ring on the ground. Colour tells the state (set in `setState`).
export class GoalMarker {
  constructor(scene) {
    this.group = new THREE.Group();
    this.pole = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 2.5, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    this.pole.rotation.x = Math.PI / 2; this.pole.position.z = 1.25;
    this.ring = new THREE.Mesh(new THREE.RingGeometry(0.5, 0.65, 32), new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide, depthTest: false }));
    this.ring.renderOrder = 9;
    this.flag = new THREE.Mesh(new THREE.PlaneGeometry(0.7, 0.4), new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide }));
    this.flag.rotation.x = Math.PI / 2; this.flag.position.set(0.35, 0, 2.3);
    this.group.add(this.pole, this.ring, this.flag);
    this.group.visible = false;
    scene.add(this.group);
  }
  show(x, y, z) { this.group.position.set(x, y, z); this.group.visible = true; }
  hide() { this.group.visible = false; }
  setColour(hex) { for (const m of [this.pole, this.ring, this.flag]) m.material.color.setHex(hex); }
}

// The route the planner returned: a polyline just above the ground, coloured per stretch (free / anomaly / frontier).
export class PlanLine {
  constructor(scene) {
    this.scene = scene;
    this.group = new THREE.Group();
    scene.add(this.group);
  }
  clear() {
    for (const o of [...this.group.children]) { this.group.remove(o); o.geometry?.dispose(); }
  }
  // waypoints: [[x, y], ...]; kinds[i] describes the stretch from waypoint i to i+1; groundAt(x, y) gives the terrain height there.
  set(waypoints, kinds, groundAt) {
    this.clear();
    if (!waypoints || waypoints.length < 2) return;
    const colours = { free: 0x19e6ff, anomaly: 0xffe14d, frontier: 0xff8a2a };
    // Draw each stretch as its own segment list, grouped by kind so there is one Line object per colour.
    const byKind = {};
    for (let i = 0; i < waypoints.length - 1; i++) {
      const k = kinds[i] in colours ? kinds[i] : 'free';
      (byKind[k] ||= []).push(waypoints[i], waypoints[i + 1]);
    }
    for (const [k, pts] of Object.entries(byKind)) {
      const arr = new Float32Array(pts.length * 3);
      pts.forEach(([x, y], j) => { arr[j * 3] = x; arr[j * 3 + 1] = y; arr[j * 3 + 2] = groundAt(x, y) + 0.2; });
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
      const line = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: colours[k], depthTest: false }));
      line.renderOrder = 8; line.frustumCulled = false;
      this.group.add(line);
    }
    const dots = new Float32Array(waypoints.length * 3);
    waypoints.forEach(([x, y], j) => { dots[j * 3] = x; dots[j * 3 + 1] = y; dots[j * 3 + 2] = groundAt(x, y) + 0.2; });
    const dg = new THREE.BufferGeometry();
    dg.setAttribute('position', new THREE.BufferAttribute(dots, 3));
    const pts = new THREE.Points(dg, new THREE.PointsMaterial({ color: 0xffffff, size: 5, sizeAttenuation: false, depthTest: false }));
    pts.renderOrder = 8; pts.frustumCulled = false;
    this.group.add(pts);
  }
}

// Outline of what the UAV's downward camera sees on the ground. The camera's horizontal field of view is 1.047 rad (60 deg) at 320 x 240 pixels
// (src/emap/models/iris_quad/model.sdf); which image side lines up with which body axis is assumed, so treat the rectangle as approximate.
export class Footprint {
  constructor(scene) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(4 * 3), 3));
    this.line = new THREE.LineLoop(geo, new THREE.LineBasicMaterial({ color: 0xffd34d, depthTest: false }));
    this.line.renderOrder = 7; this.line.frustumCulled = false;
    scene.add(this.line);
    this.halfW = Math.tan(1.047 / 2);
    this.halfH = this.halfW * 240 / 320;
  }
  update(x, y, z, yaw, groundZ) {
    const height = Math.max(z - groundZ, 0.1);
    const a = this.halfH * height, b = this.halfW * height;     // half extent along the body x axis (a) and y axis (b)
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const p = this.line.geometry.attributes.position.array;
    [[a, b], [a, -b], [-a, -b], [-a, b]].forEach(([u, v], i) => {
      p[i * 3] = x + c * u - s * v; p[i * 3 + 1] = y + s * u + c * v; p[i * 3 + 2] = groundZ + 0.1;
    });
    this.line.geometry.attributes.position.needsUpdate = true;
  }
  set visible(v) { this.line.visible = v; }
}

// A ring that follows the mouse while the goal tool is on: green = drivable, red = blocked, grey = not scanned.
export class HoverRing {
  constructor(scene) {
    this.mesh = new THREE.Mesh(new THREE.RingGeometry(0.35, 0.5, 28), new THREE.MeshBasicMaterial({ color: 0xffffff, side: THREE.DoubleSide, depthTest: false, transparent: true, opacity: 0.9 }));
    this.mesh.renderOrder = 9; this.mesh.visible = false;
    scene.add(this.mesh);
  }
  show(hit) {
    this.mesh.position.set(hit.x, hit.y, hit.z + 0.08);
    this.mesh.material.color.setHex(!hit.observed ? 0x9aa4b0 : hit.walkable ? 0x4ade80 : 0xff6b57);
    this.mesh.visible = true;
  }
  hide() { this.mesh.visible = false; }
}

// A thin vertical line from the UAV straight down to the ground, with a small ring where it would land: it makes the UAV's height readable
// from any camera angle (a flying robot over featureless ground otherwise looks glued to the floor).
export class DropLine {
  constructor(scene) {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    this.line = new THREE.Line(geo, new THREE.LineDashedMaterial({ color: 0xff9a3c, dashSize: 0.3, gapSize: 0.2, transparent: true, opacity: 0.8 }));
    this.line.frustumCulled = false;
    this.ring = new THREE.Mesh(new THREE.RingGeometry(0.25, 0.35, 24), new THREE.MeshBasicMaterial({ color: 0xff9a3c, side: THREE.DoubleSide, transparent: true, opacity: 0.8 }));
    scene.add(this.line, this.ring);
  }
  update(x, y, z, groundZ) {
    const p = this.line.geometry.attributes.position.array;
    p.set([x, y, groundZ + 0.05, x, y, z]);
    this.line.geometry.attributes.position.needsUpdate = true;
    this.line.computeLineDistances();
    this.ring.position.set(x, y, groundZ + 0.06);
  }
  set visible(v) { this.line.visible = v; this.ring.visible = v; }
}
