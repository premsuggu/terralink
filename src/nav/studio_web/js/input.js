// Turns keyboard and gamepad state into normalised stick values in [-1, 1]. The server turns those into real velocities and applies the limits
// (nav/studio/commands.py), so nothing here can command a speed - it only says "how far the stick is pushed".
//
// UAV sticks (the browser sends them as body-frame values: x forward, y LEFT, z up, wz counter-clockwise):
//   gamepad  left stick  : up/down = climb/descend, left/right = turn          right stick: up/down = forward/back, left/right = sideways
//   keyboard W/S forward/back, A/D sideways, R/F up/down, Q/E turn left/right
// UGV (manual mode):
//   gamepad  left stick up/down = forward/back, right stick left/right (or left stick left/right) = turn
//   keyboard W/S or arrows up/down = forward/back, A/D or arrows left/right = turn
// Gamepad (standard layout, as an Xbox controller reports it): axes 0/1 = left stick x/y, 2/3 = right stick x/y; buttons 0 A, 1 B, 2 X, 3 Y,
// 4 LB, 5 RB, 6 LT, 7 RT, 8 Back, 9 Start. Sticks report up as -1, so y axes are negated.

export const DEADZONE = 0.12;

// Radial dead zone (ignore tiny drift near the centre, rescale the rest to 0..1) plus a gentle response curve for fine control near the centre.
export function shapeStick(x, y, expo = 1.5) {
  const m = Math.hypot(x, y);
  if (m <= DEADZONE) return [0, 0];
  const s = Math.min(1, (m - DEADZONE) / (1 - DEADZONE));
  const k = Math.pow(s, expo) / m;
  return [x * k, y * k];
}

const clamp1 = (v) => Math.max(-1, Math.min(1, v));

// `pad` = {axes:[...], buttons:[{value}]} or null; `keys` = a Set of lower-case key names currently held; target = 'uav' | 'ugv'.
// Returns {vx, vy, vz, wz, scale, source}; all zero when nothing is pressed.
export function readSticks(pad, keys, target, baseScale = 1) {
  let vx = 0, vy = 0, vz = 0, wz = 0, source = 'none';
  const k = (n) => keys.has(n);
  if (pad) {
    const ax = (i) => (pad.axes[i] ?? 0);
    const [lx, ly] = shapeStick(ax(0), ax(1));
    const [rx, ry] = shapeStick(ax(2), ax(3));
    if (target === 'uav') { vz = -ly; wz = -lx; vy = -rx; vx = -ry; } else { vx = -ly; wz = -(Math.abs(rx) > Math.abs(lx) ? rx : lx); }
    if (vx || vy || vz || wz) source = 'gamepad';
  }
  if (source === 'none') {
    const ax1 = (pos, neg) => (k(pos) ? 1 : 0) - (k(neg) ? 1 : 0);
    if (target === 'uav') {
      vx = ax1('w', 's'); vy = ax1('a', 'd'); vz = ax1('r', 'f'); wz = ax1('q', 'e');
    } else {
      vx = ax1('w', 's') || ax1('arrowup', 'arrowdown'); wz = ax1('a', 'd') || ax1('arrowleft', 'arrowright');
    }
    if (vx || vy || vz || wz) source = 'keyboard';
  }
  // Slow mode: hold the left trigger (gamepad) or Shift (keyboard) for a fraction of the slider speed - for careful flying near obstacles.
  let slow = 1;
  if (pad && (pad.buttons[6]?.value ?? 0) > 0.05) slow = 1 - 0.75 * pad.buttons[6].value;
  if (k('shift')) slow = Math.min(slow, 0.3);
  const diag = Math.hypot(vx, vy) > 1 ? 1 / Math.hypot(vx, vy) : 1;      // keyboard diagonals must not be faster than straight
  return { vx: clamp1(vx * diag), vy: clamp1(vy * diag), vz: clamp1(vz), wz: clamp1(wz), scale: baseScale * slow, source };
}

// Edge-detect gamepad buttons: returns the names of buttons that went from released to pressed since the last call.
export class ButtonEdges {
  constructor() { this.prev = []; }
  poll(pad) {
    const names = ['a', 'b', 'x', 'y', 'lb', 'rb', 'lt', 'rt', 'back', 'start'];
    const out = [];
    names.forEach((n, i) => {
      const down = !!pad?.buttons[i]?.pressed;
      if (down && !this.prev[i]) out.push(n);
      this.prev[i] = down;
    });
    return out;
  }
}

// The first connected gamepad that reports the standard layout (browsers only expose a pad after a button is pressed on it).
export function activePad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const p of pads) if (p && p.connected) return p;
  return null;
}
