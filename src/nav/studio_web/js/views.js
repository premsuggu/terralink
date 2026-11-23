// Cameras. A "pane" is one rectangle of the screen with its own camera; the page can show 1, 2 or 4 panes at once (layouts), and each pane picks
// one of these views:
//   orbit      free camera: drag = rotate, right-drag (or Shift-drag) = pan, wheel = zoom
//   top        straight down, north up (orthographic): drag = pan, wheel = zoom. Best for choosing a goal.
//   uav_chase  behind and above the UAV, turns with it          uav_pov   what a pilot sees looking forward from the UAV
//   uav_down   the UAV's downward (scanning) camera: forward is up on the screen
//   ugv_chase  behind and above the UGV                         ugv_pov   the UGV driver's view
// THREE is the global from vendor/three.min.js.

export const VIEW_LABELS = {
  orbit: 'Free orbit', top: 'Top-down map', uav_chase: 'UAV chase', uav_pov: 'UAV first-person', uav_down: 'UAV camera (down)',
  ugv_chase: 'UGV chase', ugv_pov: 'UGV driver',
};
export const VIEW_KINDS = Object.keys(VIEW_LABELS);

// Pane rectangles as fractions of the stage [x, y, w, h] with (0, 0) at the TOP-left; later entries are drawn on top of earlier ones.
export const LAYOUTS = {
  single: [[0, 0, 1, 1]],
  inset: [[0, 0, 1, 1], [0.69, 0.03, 0.30, 0.34]],          // big view + a small second view in the corner (a minimap, typically)
  quad: [[0, 0, 0.5, 0.5], [0.5, 0, 0.5, 0.5], [0, 0.5, 0.5, 0.5], [0.5, 0.5, 0.5, 0.5]],
};
export const DEFAULT_VIEWS = { single: ['orbit'], inset: ['orbit', 'top'], quad: ['orbit', 'top', 'uav_chase', 'ugv_chase'] };

const UP = new THREE.Vector3(0, 0, 1);

export class Pane {
  constructor(kind) {
    this.persp = new THREE.PerspectiveCamera(55, 1, 0.05, 2000);
    this.ortho = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.1, 1000);
    this.persp.up.copy(UP);
    this.orbit = { az: -1.2, el: 0.65, dist: 45, target: new THREE.Vector3(0, -10, 0) };
    this.top = { cx: 0, cy: 0, half: 32 };
    this.follow = { zoom: 1 };             // wheel multiplier for the chase/POV views
    this.aspect = 1;
    this.setKind(kind);
  }

  setKind(kind) { this.kind = kind; this.camera = kind === 'top' ? this.ortho : this.persp; }

  resize(w, h) { this.aspect = Math.max(w, 1) / Math.max(h, 1); this.pixelHeight = h; }

  // ---- mouse / touch input ----
  drag(dx, dy, pan) {
    if (this.kind === 'orbit') {
      if (pan) {
        // move the target along the screen's right and "forward on the ground" directions, in proportion to the zoom distance
        const k = this.orbit.dist * 0.0016, c = Math.cos(this.orbit.az), s = Math.sin(this.orbit.az);
        this.orbit.target.x += (-s * dx + c * dy) * k * -1; this.orbit.target.y += (c * dx + s * dy) * k * -1;
      } else {
        this.orbit.az -= dx * 0.006;
        this.orbit.el = Math.max(0.05, Math.min(1.5, this.orbit.el + dy * 0.006));
      }
    } else if (this.kind === 'top') {
      const k = (2 * this.top.half) / (this.pixelHeight || 600);
      this.top.cx -= dx * k; this.top.cy += dy * k;
    }
  }
  zoom(amount) {
    const f = Math.exp(amount * 0.0012);
    if (this.kind === 'orbit') this.orbit.dist = Math.max(2, Math.min(400, this.orbit.dist * f));
    else if (this.kind === 'top') this.top.half = Math.max(3, Math.min(120, this.top.half * f));
    else this.follow.zoom = Math.max(0.3, Math.min(6, this.follow.zoom * f));
  }
  focus(x, y) {
    this.orbit.target.set(x, y, 0); this.top.cx = x; this.top.cy = y;
  }

  // Position the camera for this frame. `s` = {uav, ugv} smoothed poses ({x, y, z, yaw}, either may be null); `vs` = vehicle display scale.
  update(s, vs) {
    const cam = this.camera;
    switch (this.kind) {
      case 'orbit': {
        const o = this.orbit, ce = Math.cos(o.el);
        cam.position.set(o.target.x + o.dist * ce * Math.cos(o.az), o.target.y + o.dist * ce * Math.sin(o.az), o.target.z + o.dist * Math.sin(o.el));
        cam.up.copy(UP); cam.lookAt(o.target); cam.fov = 55;
        break;
      }
      case 'top': {
        const t = this.top, a = this.aspect;
        cam.left = -t.half * a; cam.right = t.half * a; cam.top = t.half; cam.bottom = -t.half;
        cam.position.set(t.cx, t.cy, 300); cam.up.set(0, 1, 0); cam.lookAt(t.cx, t.cy, 0);
        break;
      }
      default: {
        const isUav = this.kind.startsWith('uav'), p = isUav ? s.uav : s.ugv;
        if (!p) { cam.position.set(0, -20, 15); cam.up.copy(UP); cam.lookAt(0, 0, 0); break; }
        const c = Math.cos(p.yaw), sn = Math.sin(p.yaw), z = this.follow.zoom;
        cam.up.copy(UP);
        if (this.kind.endsWith('chase')) {
          const back = (isUav ? 3.2 : 2.6) * vs * z, up = (isUav ? 1.6 : 1.5) * vs * z;
          cam.position.set(p.x - c * back, p.y - sn * back, p.z + up);
          cam.lookAt(p.x + c * 1.5 * vs, p.y + sn * 1.5 * vs, p.z + 0.2 * vs);
          cam.fov = 60;
        } else if (this.kind === 'uav_pov') {
          cam.position.set(p.x + c * 0.2 * vs, p.y + sn * 0.2 * vs, p.z + 0.02 * vs);
          cam.lookAt(cam.position.x + c * 10, cam.position.y + sn * 10, cam.position.z - 1.5);     // looking slightly down, like the scan camera's tilt
          cam.fov = 70;
        } else if (this.kind === 'uav_down') {
          cam.position.set(p.x, p.y, p.z - 0.05);
          cam.up.set(c, sn, 0);                                  // the UAV's forward direction is the top of the image
          cam.lookAt(p.x, p.y, p.z - 10);
          cam.fov = 46.8;                                        // vertical field of view of the 60 deg x 320x240 scanning camera
        } else {                                                 // ugv_pov
          cam.position.set(p.x + c * 0.3 * vs * z, p.y + sn * 0.3 * vs * z, p.z + 0.42 * vs);
          cam.lookAt(cam.position.x + c * 10, cam.position.y + sn * 10, cam.position.z - 0.5);
          cam.fov = 75;
        }
      }
    }
    if (cam.isPerspectiveCamera) cam.aspect = this.aspect;
    cam.updateProjectionMatrix();
  }

  // Does this view sit INSIDE the given vehicle (so that vehicle's own model must be hidden while drawing it)?
  hidesVehicle() {
    return this.kind === 'uav_pov' || this.kind === 'uav_down' ? 'uav' : this.kind === 'ugv_pov' ? 'ugv' : null;
  }
}
