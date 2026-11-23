// TerraLink Studio - the browser app. Connects to the studio server (one websocket), draws the world in 1, 2 or 4 camera panes, and turns the
// keyboard / Xbox controller into UAV and UGV commands. All limits are enforced by the SERVER; this page only sends normalised stick values.
// Message formats: nav/studio/commands.py (client -> server) and nav/studio/protocol.py + studio_node.py (server -> client).
import { decodeMap, MapGrid } from './protocol.js';
import { Terrain } from './terrain.js';
import { makeUav, makeUgv, makeBeacon, Trail, GoalMarker, PlanLine, Footprint, HoverRing, DropLine } from './vehicles.js';
import { Scenery } from './scenery.js';
import { Pane, VIEW_KINDS, VIEW_LABELS, LAYOUTS, DEFAULT_VIEWS } from './views.js';
import { readSticks, ButtonEdges, activePad } from './input.js';

const $ = (id) => document.getElementById(id);
const stage = $('stage'), canvas = $('gl'), panesEl = $('panes');

// ---------------------------------------------------------------- preferences (remembered per browser; never required to work)
const prefs = { layout: 'inset', views: {}, colour: 'height', scale: 4, speed: 0.7, trails: true, footprint: true, world: true, scanned: true, follow: 'none', helpSeen: false };
try { Object.assign(prefs, JSON.parse(localStorage.getItem('terralink.prefs') || '{}')); } catch (e) { /* private window etc.: keep defaults */ }
const savePrefs = () => { try { localStorage.setItem('terralink.prefs', JSON.stringify(prefs)); } catch (e) { /* ignore */ } };

// ---------------------------------------------------------------- 3D scene
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
} catch (e) {
  stage.insertAdjacentHTML('beforeend', '<div style="position:absolute;inset:0;display:grid;place-items:center;color:#ff8a80">WebGL is not available in this browser, so the 3D view cannot start.</div>');
  throw e;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);                      // Z is up, like the ROS map frame
const scene = new THREE.Scene();
{ // sky: a vertical gradient painted once on a tiny canvas
  const c = document.createElement('canvas'); c.width = 2; c.height = 256;
  const g = c.getContext('2d'), grad = g.createLinearGradient(0, 0, 0, 256);
  grad.addColorStop(0, '#1b2a41'); grad.addColorStop(1, '#8aa4bf');
  g.fillStyle = grad; g.fillRect(0, 0, 2, 256);
  scene.background = new THREE.CanvasTexture(c);
}
const hemi = new THREE.HemisphereLight(0xdfeeff, 0x4a5563, 0.6); hemi.position.set(0, 0, 1); scene.add(hemi);
const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(30, -20, 60); scene.add(sun);

const terrain = new Terrain(scene);
const scenery = new Scenery(scene);               // the real site's boxes (containers, fences): for the human only, the robots never see them
const uav = makeUav(), ugv = makeUgv();
scene.add(uav, ugv);
uav.visible = ugv.visible = false;                           // shown when the first pose arrives
const uavBeacon = makeBeacon(0xff7a1a), ugvBeacon = makeBeacon(0xf2c200);
scene.add(uavBeacon, ugvBeacon);
const uavTrail = new Trail(scene, 0xff9a3c), ugvTrail = new Trail(scene, 0xf2e04a);
const dropLine = new DropLine(scene);
const goalMarker = new GoalMarker(scene), planLine = new PlanLine(scene), footprint = new Footprint(scene), hoverRing = new HoverRing(scene);
footprint.visible = false; dropLine.visible = false;

// ---------------------------------------------------------------- application state
const S = {
  ws: null, connected: false, hello: null, state: null, grid: null, mapVersion: -1, firstMapSeen: false,
  target: 'uav',                       // who the sticks control: 'uav' | 'ugv' (manual UGV)
  goalTool: false, estop: false, lastGoalKey: null, lastNav: 'none',
  smooth: { uav: null, ugv: null },    // poses drawn this frame (eased towards the latest state so 10 Hz updates look smooth)
  latest: { uav: null, ugv: null },
  keys: new Set(), pointer: null, hover: null, framed: false, follow: 'none', climbing: null,
};
const panes = [];                      // Pane objects, kept across layout changes so each keeps its camera
let paneEls = [];                      // the DOM for the panes of the current layout

// ---------------------------------------------------------------- small helpers
function toast(text, level = '') {
  const d = document.createElement('div'); d.textContent = text; if (level) d.className = level;
  $('toast').appendChild(d);
  setTimeout(() => d.remove(), level === 'bad' ? 6000 : 3800);
}
function addLog(level, text) {
  const d = document.createElement('div'); d.textContent = text; if (level === 'warn') d.className = 'warn';
  const log = $('log'); log.appendChild(d);
  while (log.children.length > 6) log.firstChild.remove();
  setTimeout(() => d.remove(), 15000);
}
const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const fmt = (v, d = 1) => (v == null || Number.isNaN(v) ? '–' : v.toFixed(d));
const deg = (r) => ((r * 180 / Math.PI) % 360 + 360) % 360;
function send(obj) { if (S.ws && S.ws.readyState === 1) S.ws.send(JSON.stringify(obj)); }

// ---------------------------------------------------------------- websocket
let retry = 500;
function connect() {
  const ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws');
  ws.binaryType = 'arraybuffer';
  S.ws = ws;
  ws.onopen = () => { S.connected = true; retry = 500; setConn('connected', 'good'); updateCoach(); };
  ws.onclose = () => {
    S.connected = false; setConn('disconnected – retrying', 'bad'); updateCoach();
    setTimeout(connect, retry); retry = Math.min(retry * 1.7, 5000);
  };
  ws.onmessage = (ev) => (typeof ev.data === 'string' ? onText(JSON.parse(ev.data)) : onMapFrame(ev.data));
}
function setChip(id, text, cls = '') { const e = $(id); e.textContent = text; e.className = 'tag' + (cls ? ' ' + cls : ''); }
// the connection tag has a coloured dot in front of its text
function setConn(text, cls) { const e = $('chip-conn'); e.className = 'tag ' + cls; e.querySelector('span').textContent = text; }
const setText = (id, text) => { const e = $(id); if (e.textContent !== text) e.textContent = text; };

function onText(m) {
  if (m.t === 'hello') onHello(m);
  else if (m.t === 'state') onState(m);
  else if (m.t === 'plan') onPlan(m.plan);
  else if (m.t === 'settings') showSettings(m.settings);
  else if (m.t === 'log') { addLog(m.level, m.msg); if (m.level === 'warn') toast(m.msg, 'warn'); }
  else if (m.t === 'error') toast('Server: ' + m.msg, 'bad');
}

function onHello(m) {
  S.hello = m;
  const live = m.map_source === 'live';
  S.live = live;
  setChip('chip-site', m.site.replace(/_/g, ' '));
  setChip('chip-source', live ? 'map: live scan' : `map: saved (${m.map_source.replace(/^.*?_(uavview|groundtruth|uavscan)$/, '$1')})`, live ? 'good' : 'warn');
  $('chip-source').title = live ? 'The elevation map is built live from what the UAV scans' : 'A saved map is used: no scanning needed, go straight to planning';
  document.title = `TerraLink Studio – ${m.site}`;
  document.querySelectorAll('.live-only').forEach((e) => { e.hidden = !live; });     // take-off only makes sense when you build the map
  buildSettings(m);
  scenery.set(m.scenery || []);
  $('opt-world').closest('label').hidden = !(m.scenery && m.scenery.length);
  frameCameras();
  updateCoach();
}

// ---- settings card (P5): sliders show/ask for the run-time limits. The SERVER clamps every value to its hard caps and answers with a
// "settings" event holding what it really applied, so the sliders always show the truth (never what we merely asked for).
const SET_UNITS = { max_xy: 'm/s', max_z: 'm/s', max_yaw: 'rad/s', ceiling: 'm', ugv_max_lin: 'm/s', ugv_max_ang: 'rad/s' };
const PRESETS = {
  gentle: { max_xy: 1.5, max_z: 0.8, max_yaw: 0.6, ceiling: 8, ugv_max_lin: 0.5, ugv_max_ang: 0.8 },
  normal: { max_xy: 3, max_z: 1.5, max_yaw: 1, ceiling: 14, ugv_max_lin: 1, ugv_max_ang: 1.5 },
  fast:   { max_xy: 5, max_z: 2.5, max_yaw: 2, ceiling: 14, ugv_max_lin: 1, ugv_max_ang: 1.5 },
};
function buildSettings(m) {
  document.querySelectorAll('[data-set]').forEach((el) => {
    const cap = m.hard_caps && m.hard_caps[el.dataset.set];
    if (cap) { el.min = cap[0]; el.max = cap[1]; }
  });
  if (m.settings) showSettings(m.settings);
}
function showSettings(st) {
  S.settings = st;
  for (const [k, v] of Object.entries(st)) {
    const el = document.querySelector(`[data-set="${k}"]`);
    if (el && document.activeElement !== el) el.value = v;
    setText('val-' + k, `${(+v).toFixed(k === 'ceiling' ? 0 : 1)} ${SET_UNITS[k]}`);
  }
  let hit = null;
  for (const [name, p] of Object.entries(PRESETS)) if (Object.entries(p).every(([k, v]) => Math.abs((st[k] ?? NaN) - v) < 0.05)) hit = name;
  document.querySelectorAll('#seg-preset button').forEach((b) => b.classList.toggle('on', b.dataset.preset === hit));
}
document.querySelectorAll('[data-set]').forEach((el) => el.addEventListener('change', () => send({ t: 'settings', [el.dataset.set]: +el.value })));
document.querySelectorAll('#seg-preset button').forEach((b) => b.addEventListener('click', () => send({ t: 'settings', ...PRESETS[b.dataset.preset] })));

function onMapFrame(buf) {
  let decoded;
  try { decoded = decodeMap(buf); } catch (e) { addLog('warn', 'bad map frame: ' + e.message); return; }
  if (decoded.header.v < S.mapVersion) return;                 // an older frame arriving late
  S.mapVersion = decoded.header.v;
  const grid = new MapGrid(decoded);
  S.grid = grid;
  terrain.update(grid);
  updateReach();
  frameCameras();
}

// Which scanned drivable cells can the UGV actually get to? A flood fill from the UGV over observed+walkable cells (8-neighbours). A container roof is
// scanned and flat, so it looks drivable, but nothing connects it to the ground: such a goal gets "no route". Showing that BEFORE the click
// saves a confusing failure. (The planner also keeps a safety margin round obstacles, so "connected" is necessary, not sufficient.)
function updateReach() {
  const g = S.grid, u = S.latest.ugv;
  if (!g || !u || g.h < 2) { S.reach = null; terrain.setReach(null); return; }
  const n = g.h * g.w, reach = new Uint8Array(n), queue = new Int32Array(n);
  let qh = 0, qt = 0;
  const rc = Math.ceil(0.6 / g.res);                      // the planner clears a start circle round the UGV: seed every scanned cell in it
  const c0 = Math.round((u.x - g.x0) / g.res), r0 = Math.round((u.y - g.y0) / g.res);
  for (let dr = -rc; dr <= rc; dr++) for (let dc = -rc; dc <= rc; dc++) {
    const r = r0 + dr, c = c0 + dc;
    if (r < 0 || c < 0 || r >= g.h || c >= g.w || dr * dr + dc * dc > rc * rc) continue;
    const i = r * g.w + c;
    if (g.isObserved(i) && !reach[i]) { reach[i] = 1; queue[qt++] = i; }
  }
  while (qh < qt) {
    const i = queue[qh++], r = (i / g.w) | 0, c = i - r * g.w;
    for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
      const rr = r + dr, cc = c + dc;
      if (rr < 0 || cc < 0 || rr >= g.h || cc >= g.w) continue;
      const j = rr * g.w + cc;
      if (!reach[j] && g.isObserved(j) && g.isWalkable(j)) { reach[j] = 1; queue[qt++] = j; }
    }
  }
  S.reach = reach; terrain.setReach(reach);
}

function onState(s) {
  S.state = s;
  // chips
  const rtf = s.rtf == null ? '' : `  ×${s.rtf.toFixed(2)}`;
  setChip('chip-time', `sim ${s.sim_t.toFixed(0)} s${rtf}`, s.rtf != null && s.rtf < 0.4 ? 'warn' : '');
  setText('chip-map', `${s.map.area_m2.toFixed(0)} m²`);
  if (performance.now() > (S.estopLocalUntil || 0)) S.estop = !!s.cmd.estop;      // (see toggleEstop)
  $('btn-estop').classList.toggle('active', S.estop);
  $('btn-estop').innerHTML = S.estop ? '■ STOPPED – click to release <small>Space</small>' : '■ EMERGENCY STOP <small>Space</small>';
  const banner = $('banner');
  banner.hidden = !S.estop;
  if (S.estop) banner.textContent = 'EMERGENCY STOP is on – nothing moves. Press Space to release.';
  S.climbing = s.cmd.climb_to;
  $('btn-takeoff').disabled = S.estop || S.climbing != null || !!s.cmd.autoscan;
  const scanning = !!s.cmd.autoscan;                       // the same button starts and stops the sweep
  setText('btn-autoscan', scanning ? '■ Stop auto-scan' : '⟳ Auto-scan the site');
  $('btn-autoscan').disabled = S.estop || s.mode === 'manual';
  $('limit-note').hidden = !s.uav_limited;
  // keep the "you control" selector in step with the server (e.g. after a page reload while manual mode was on)
  const serverTarget = s.mode === 'manual' ? 'ugv' : 'uav';
  if (serverTarget !== S.target) setTarget(serverTarget, false);
  // poses (the drawn ones ease towards these in the frame loop)
  if (s.uav) { S.latest.uav = s.uav; uavTrail.add(s.uav.x, s.uav.y, s.uav.z); }
  if (s.ugv) {
    const z = terrain.heightAt(s.ugv.x, s.ugv.y, 0);
    S.latest.ugv = { ...s.ugv, z };
    ugvTrail.add(s.ugv.x, s.ugv.y, z + 0.1);
  }
  // goal flag
  const gk = s.goal ? `${s.goal.x.toFixed(2)},${s.goal.y.toFixed(2)}` : null;
  if (gk !== S.lastGoalKey) { S.lastGoalKey = gk; S.noRouteToasted = false; }
  if (s.goal) {
    goalMarker.show(s.goal.x, s.goal.y, terrain.heightAt(s.goal.x, s.goal.y, 0));
    goalMarker.setColour({ planning: 0xffd34d, driving: 0x19e6ff, no_route: 0xff6b57, arrived: 0x4ade80 }[s.nav] || 0xffffff);
  } else goalMarker.hide();
  if (s.nav === 'arrived' && S.lastNav !== 'arrived') toast('The UGV reached the goal.');
  if (s.nav === 'no_route' && !S.noRouteToasted) { S.noRouteToasted = true; toast("No route to that goal. The UGV's own spot and the whole way to the goal must be scanned ground: fly the UAV over them, or pick another spot.", 'bad'); }
  S.lastNav = s.nav;
  updateCoach();
  // text readouts
  const u = s.uav, g = s.ugv;
  setText('hud-uav', u ? `${fmt(u.x)}, ${fmt(u.y)} · alt ${fmt(u.z)} m · ${deg(u.yaw).toFixed(0)}°` : 'waiting for pose…');
  setText('hud-ugv', g ? `${fmt(g.x)}, ${fmt(g.y)} · ${deg(g.yaw).toFixed(0)}°` : 'waiting for pose…');
  const dist = s.goal && g ? Math.hypot(g.x - s.goal.x, g.y - s.goal.y) : null;
  setText('hud-goal', s.goal ? `${fmt(s.goal.x)}, ${fmt(s.goal.y)}` + (dist != null ? `  (${dist.toFixed(1)} m)` : '') : 'none');
  const missionText = { none: 'No goal yet. Press Set goal, then click the map.', planning: 'Planning a route…', no_route: 'No route found.',
    driving: dist != null ? `Driving – ${dist.toFixed(1)} m to go.` : 'Driving…', arrived: 'Arrived at the goal.' }[s.nav] || s.nav;
  setText('mission', s.mode === 'manual' ? 'UGV is driven by hand (planner paused).' : missionText);
  if (!S.framed) frameCameras();
}

function onPlan(plan) {
  if (!plan || !plan.valid) { planLine.clear(); return; }
  planLine.set(plan.waypoints, plan.kinds, (x, y) => terrain.heightAt(x, y, 0));
}

// ---------------------------------------------------------------- first view: close to the robots, with the whole site in the map pane
// Runs until it has what it needs (a robot pose, plus the site's extent from the scenery or the map), then never again: after that the cameras
// are the person's. (Framing the whole 56 m site made the 0.5 m robots invisible dots, so the free-orbit camera starts near the UGV.)
function siteBounds() {
  return scenery.bounds || (S.grid && S.grid.bounds) || null;
}
function frameCameras() {
  if (S.framed) return;
  const home = S.latest.ugv || S.latest.uav;
  if (!home) return;                                  // wait for the first pose
  const b = siteBounds();
  S.framed = true;
  for (const p of panes) {
    p.orbit.target.set(home.x, home.y, 0);
    p.orbit.dist = 24; p.orbit.el = 0.55; p.orbit.az = -Math.PI / 2;      // camera south of the UGV, looking north over it
    if (b) { p.top.cx = (b.xmin + b.xmax) / 2; p.top.cy = (b.ymin + b.ymax) / 2; p.top.half = Math.max(b.xmax - b.xmin, b.ymax - b.ymin) / 2 * 1.06; }
    else { p.top.cx = home.x; p.top.cy = home.y; p.top.half = 30; }
  }
}

// ---------------------------------------------------------------- the coach: one box that always says what to do next
// Driven only by state the server already sends, so it is never out of date. Each case: tone (colour), a small heading, a title, a sentence of
// help and optionally one button that does the next thing for you.
function coachNow() {
  const s = S.state;
  if (!S.connected || !s) return { tone: 'warn', eyebrow: 'CONNECTING', title: 'Connecting to the studio…', body: 'If this stays, check that the studio is still running in its terminal.' };
  if (S.estop) return { tone: 'bad', eyebrow: 'STOPPED', title: 'Emergency stop is on', body: 'Nothing will move and your controls are ignored until you release it: press Space (or A on the controller).', action: ['Release the stop', toggleEstop] };
  if (s.mode === 'manual') return { tone: 'warn', eyebrow: 'MANUAL', title: 'You are driving the UGV', body: 'W/S drive, A/D steer (controller: left stick drives, right stick steers). The planner is paused. Switch back to give it goals.', action: ['Back to planner (UAV)', () => setTarget('uav')] };
  if (s.cmd.autoscan) return { tone: 'info', eyebrow: 'AUTO-SCAN', title: `Scanning the site… leg ${Math.min(s.cmd.autoscan.i + 1, s.cmd.autoscan.n)} of ${s.cmd.autoscan.n}`, body: 'The UAV is flying a back-and-forth sweep by itself. This takes several minutes. Watch the map fill in. Touch any flight key (or the e-stop) to take over.', action: ['Stop auto-scan', stopAutoscan] };
  if (S.goalTool) return { tone: 'info', eyebrow: 'SET GOAL', title: 'Click the destination', body: 'Click a green spot in any view. Red = blocked, grey = not scanned yet. Esc cancels.' };
  if (s.nav === 'planning') return { tone: 'info', eyebrow: 'PLANNING', title: 'Finding a route…', body: 'The planner is working out a path over the scanned map.' };
  if (s.nav === 'driving') return { tone: 'info', eyebrow: 'DRIVING', title: 'The UGV is on its way', body: 'The cyan line is its route. You can keep flying the UAV, switch views, or cancel the goal.', action: ['Cancel goal', cancelGoal] };
  if (s.nav === 'arrived') return { tone: 'good', eyebrow: 'DONE', title: 'The UGV arrived', body: 'Pick another goal to keep going.', action: ['Set another goal', () => setGoalTool(true)] };
  if (s.nav === 'no_route') {
    const why = S.live ? "The UGV's own spot and the whole way to the goal must be scanned ground, joined up without gaps (a container roof is scanned but cut off). Fly slowly at 3-4 m over the whole way, or pick another spot. Map colours > drivable shows blue for cut-off ground." : 'That spot is probably blocked or walled off. Pick another goal.';
    return { tone: 'bad', eyebrow: 'NO ROUTE', title: 'No route to that goal', body: why, action: ['Pick another goal', () => setGoalTool(true)] };
  }
  // no goal yet
  if (!S.live) return { tone: 'info', eyebrow: 'NEXT STEP', title: 'Pick a goal for the UGV', body: 'This is a saved map, so there is nothing to scan. Press Set goal, then click a green spot: the UGV plans a route and drives there.', action: ['Set goal', () => setGoalTool(true)] };
  const u = s.uav;
  if (s.cmd.climb_to != null) return { tone: 'info', eyebrow: 'TAKING OFF', title: `Climbing to ${s.cmd.climb_to.toFixed(0)} m…`, body: 'Move the climb stick (R / F) to take over at any time.' };
  if (!u || u.z < 0.6) return { tone: 'info', eyebrow: 'STEP 1 · TAKE OFF', title: 'The UAV is on the ground', body: 'Press Auto-scan to let the UAV map the site by itself, or Take off and fly it yourself. Fly over the area the UGV must cross: the map appears under the UAV as it scans.', action: ['Auto-scan the site', autoscan] };
  const ugvScanned = S.grid && S.latest.ugv && S.grid.cellIndex(S.latest.ugv.x, S.latest.ugv.y) >= 0 && S.grid.isObserved(S.grid.cellIndex(S.latest.ugv.x, S.latest.ugv.y));
  if (s.map.area_m2 < 40 || !ugvScanned) {
    return { tone: 'info', eyebrow: 'STEP 1 · SCAN', title: 'Scan the ground', body: (!ugvScanned ? 'Fly over the yellow UGV first (its own spot must be scanned), then ' : 'Keep flying ') + 'over the ground it should cross. Scanned ground turns coloured; the dark boxes are the real site, which the robots cannot see yet.' };
  }
  return { tone: 'good', eyebrow: 'STEP 2 · GOAL', title: 'Pick a goal for the UGV', body: `${s.map.area_m2.toFixed(0)} m² scanned. Press Set goal, then click a green spot on scanned ground. Scan more any time to extend the map.`, action: ['Set goal', () => setGoalTool(true)] };
}
let lastCoachKey = '';
function updateCoach() {
  const c = coachNow();
  const key = JSON.stringify([c.tone, c.eyebrow, c.title, c.body, c.action && c.action[0]]);
  if (key === lastCoachKey) return;
  lastCoachKey = key;
  $('coach').dataset.tone = c.tone;
  $('coach-eyebrow').textContent = c.eyebrow; $('coach-title').textContent = c.title; $('coach-body').textContent = c.body;
  const btn = $('coach-action');
  btn.hidden = !c.action;
  if (c.action) { btn.textContent = c.action[0]; S.coachAction = c.action[1]; }
}
$('coach-action').addEventListener('click', (e) => { if (S.coachAction) S.coachAction(); e.currentTarget.blur(); });

// ---------------------------------------------------------------- panes and layouts
function ensurePanes(n) { while (panes.length < n) panes.push(new Pane(DEFAULT_VIEWS.quad[panes.length] || 'orbit')); }

function setLayout(name) {
  prefs.layout = name; savePrefs();
  const rects = LAYOUTS[name], kinds = prefs.views[name] || DEFAULT_VIEWS[name];
  ensurePanes(rects.length);
  panesEl.innerHTML = ''; paneEls = [];
  rects.forEach((r, i) => {
    const pane = panes[i]; pane.setKind(kinds[i] || 'orbit');
    const el = document.createElement('div'); el.className = 'pane';
    el.style.cssText = `left:${r[0] * 100}%;top:${r[1] * 100}%;width:${r[2] * 100}%;height:${r[3] * 100}%`;
    const opts = VIEW_KINDS.map((k) => `<option value="${k}"${k === pane.kind ? ' selected' : ''}>${VIEW_LABELS[k]}</option>`).join('');
    el.innerHTML = `<div class="hit"></div><div class="bar"><select title="What this pane shows">${opts}</select>` +
      `<button data-f="ugv" title="Move this view's centre to the UGV">to UGV</button><button data-f="uav" title="Move this view's centre to the UAV">to UAV</button><button data-f="map" title="Show the whole site">whole site</button></div>`;
    el.querySelector('select').addEventListener('change', (e) => { pane.setKind(e.target.value); (prefs.views[name] ||= [])[i] = e.target.value; savePrefs(); e.target.blur(); });
    el.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => focusPane(pane, b.dataset.f)));
    wirePane(el.querySelector('.hit'), pane);
    panesEl.appendChild(el); paneEls.push(el);
  });
  document.querySelectorAll('#seg-layout button').forEach((b) => b.classList.toggle('on', b.dataset.layout === name));
  updateGoalToolUi();
  resize();
}

function focusPane(pane, what) {
  if (what === 'map') {
    const b = siteBounds(); if (!b) return;
    pane.focus((b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2);
    const size = Math.max(b.xmax - b.xmin, b.ymax - b.ymin);
    pane.top.half = size / 2 * 1.06; pane.orbit.dist = size * 0.9;
    return;
  }
  const p = what === 'uav' ? S.smooth.uav : S.smooth.ugv;
  if (p) pane.focus(p.x, p.y);
}

function setMainView(kind) {         // main pane = pane 0
  const pane = panes[0]; pane.setKind(kind);
  (prefs.views[prefs.layout] ||= [])[0] = kind; savePrefs();
  const sel = paneEls[0]?.querySelector('select'); if (sel) sel.value = kind;
}
function cycleMainView(step) {
  const i = VIEW_KINDS.indexOf(panes[0].kind);
  setMainView(VIEW_KINDS[(i + step + VIEW_KINDS.length) % VIEW_KINDS.length]);
}
function cycleLayout() {
  const names = Object.keys(LAYOUTS);
  setLayout(names[(names.indexOf(prefs.layout) + 1) % names.length]);
}

// mouse: rotate/pan/zoom, hover cursor, and goal picking
function wirePane(el, pane) {
  let drag = null;
  el.addEventListener('pointerdown', (e) => { el.setPointerCapture(e.pointerId); drag = { x: e.clientX, y: e.clientY, btn: e.button, moved: 0 }; });
  el.addEventListener('pointermove', (e) => {
    S.pointer = { pane, el, x: e.clientX, y: e.clientY };
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    drag.x = e.clientX; drag.y = e.clientY; drag.moved += Math.abs(dx) + Math.abs(dy);
    pane.drag(dx, dy, drag.btn === 2 || e.shiftKey);
  });
  el.addEventListener('pointerup', (e) => {
    if (drag && drag.moved < 5 && drag.btn === 0 && S.goalTool) pickGoal(pane, el, e.clientX, e.clientY);
    drag = null;
  });
  el.addEventListener('pointerleave', () => { S.pointer = null; S.hover = null; hoverRing.hide(); $('hud-cursor').textContent = '–'; });
  el.addEventListener('wheel', (e) => { e.preventDefault(); pane.zoom(e.deltaY); }, { passive: false });
  el.addEventListener('contextmenu', (e) => e.preventDefault());
}

// Where does the mouse point in the world? Returns {x, y, z, observed, walkable} or null.
const raycaster = new THREE.Raycaster();
function pickWorld(pane, el, clientX, clientY) {
  const r = el.getBoundingClientRect();
  raycaster.setFromCamera({ x: ((clientX - r.left) / r.width) * 2 - 1, y: -((clientY - r.top) / r.height) * 2 + 1 }, pane.camera);
  const { origin: o, direction: d } = raycaster.ray;
  if (S.grid) return S.grid.raycast(o.x, o.y, o.z, d.x, d.y, d.z);
  if (d.z >= 0) return null;                                     // no map yet: intersect the z = 0 plane
  const t = -o.z / d.z;
  return { x: o.x + d.x * t, y: o.y + d.y * t, z: 0, observed: false, walkable: false };
}

function pickGoal(pane, el, cx, cy) {
  const hit = pickWorld(pane, el, cx, cy);
  if (!hit) { toast('Click on the ground.', 'warn'); return; }
  if (S.target === 'ugv') { toast('You are driving the UGV by hand. Switch to UAV control so the planner can drive it.', 'warn'); return; }
  if (hit.observed && !hit.walkable) { toast('That spot is blocked (red). Pick a green spot.', 'warn'); return; }
  if (hit.observed && S.reach && S.grid) { const i = S.grid.cellIndex(hit.x, hit.y); if (i >= 0 && !S.reach[i]) toast('That spot is drivable but cut off from the UGV by scanned obstacles (a container roof or an island): the planner will probably find no route. Sending it anyway - or pick another green spot.', 'warn'); }
  if (!hit.observed) toast('That ground has not been scanned yet - the planner may find no route.', 'warn');
  send({ t: 'goal', x: +hit.x.toFixed(2), y: +hit.y.toFixed(2) });
  setGoalTool(false);
}

// ---------------------------------------------------------------- controls
function setTarget(target, notify = true) {
  S.target = target;
  document.querySelectorAll('#seg-target button').forEach((b) => b.classList.toggle('on', b.dataset.target === target));
  $('keys-uav').hidden = target !== 'uav'; $('keys-ugv').hidden = target !== 'ugv';
  $('btn-takeoff').disabled = target !== 'uav' || S.estop;
  updateCoach();
  if (notify) {
    send({ t: 'mode', mode: target === 'ugv' ? 'manual' : 'auto' });
    if (target === 'ugv') toast('You now drive the UGV. The planner is paused and any goal was cancelled.');
  }
}
function setGoalTool(on) {
  S.goalTool = on; updateGoalToolUi(); updateCoach();
}
function updateGoalToolUi() {
  $('btn-goal').classList.toggle('on', S.goalTool);
  paneEls.forEach((el) => el.classList.toggle('goaltool', S.goalTool));
}
// The button/key flips the stop on the page's OWN idea of the state at once, and state messages that were already in flight are ignored for a moment.
// Otherwise a slow page (software rendering) could still show the old state when Space is pressed twice quickly, and send "estop" twice.
function toggleEstop() {
  S.estop = !S.estop;
  S.estopLocalUntil = performance.now() + 700;
  send({ t: S.estop ? 'estop' : 'estop_release' });
  updateCoach();
}
function cancelGoal() { send({ t: 'cancel_goal' }); setGoalTool(false); }
function autoscan() {
  if (S.estop) { toast('The emergency stop is on. Release it (Space) first.', 'warn'); return; }
  send({ t: 'autoscan', alt: 5 });
}
function stopAutoscan() { send({ t: 'autoscan_cancel' }); }
function takeoff() {
  if (S.estop) { toast('The emergency stop is on. Release it (Space) first.', 'warn'); return; }
  send({ t: 'takeoff', alt: 6 });
}

$('seg-target').addEventListener('click', (e) => { if (e.target.dataset.target) setTarget(e.target.dataset.target); });
$('seg-layout').addEventListener('click', (e) => { if (e.target.dataset.layout) setLayout(e.target.dataset.layout); });
$('seg-colour').addEventListener('click', (e) => {
  const c = e.target.dataset.colour; if (!c) return;
  prefs.colour = c; savePrefs(); terrain.setMode(c);
  document.querySelectorAll('#seg-colour button').forEach((b) => b.classList.toggle('on', b.dataset.colour === c));
});
$('btn-goal').addEventListener('click', () => setGoalTool(!S.goalTool));
$('btn-cancel').addEventListener('click', cancelGoal);
$('btn-takeoff').addEventListener('click', takeoff);
$('btn-autoscan').addEventListener('click', () => (S.state && S.state.cmd.autoscan ? stopAutoscan() : autoscan()));
$('btn-estop').addEventListener('click', toggleEstop);
$('btn-help').addEventListener('click', () => { $('help').hidden = false; });
function closeHelp() { $('help').hidden = true; prefs.helpSeen = true; savePrefs(); }
$('btn-help-close').addEventListener('click', closeHelp);
$('help').addEventListener('click', (e) => { if (e.target.id === 'help') closeHelp(); });
$('seg-follow').addEventListener('click', (e) => {
  const f = e.target.dataset.follow; if (!f) return;
  S.follow = f; document.querySelectorAll('#seg-follow button').forEach((b) => b.classList.toggle('on', b.dataset.follow === f));
});
$('opt-world').addEventListener('change', (e) => { prefs.world = e.target.checked; savePrefs(); scenery.visible = prefs.world; });
$('opt-scanned').addEventListener('change', (e) => { prefs.scanned = e.target.checked; savePrefs(); terrain.setScannedVisible(prefs.scanned); });
$('rng-speed').addEventListener('input', (e) => { prefs.speed = +e.target.value; savePrefs(); $('val-speed').textContent = Math.round(prefs.speed * 100) + '%'; });
$('rng-speed').addEventListener('change', (e) => e.target.blur());
$('rng-scale').addEventListener('input', (e) => { prefs.scale = +e.target.value; savePrefs(); $('val-scale').textContent = '×' + prefs.scale; });
$('rng-scale').addEventListener('change', (e) => e.target.blur());
$('opt-trails').addEventListener('change', (e) => { prefs.trails = e.target.checked; savePrefs(); });
$('opt-footprint').addEventListener('change', (e) => { prefs.footprint = e.target.checked; savePrefs(); });

// A clicked button keeps keyboard focus, and Space then also "clicks" it - which would e-stop AND press whatever button was used last. So every
// button gives its focus back after a click (the page's keys are global) and Space never reaches a button on its own.
document.addEventListener('click', (e) => { const b = e.target.closest && e.target.closest('button'); if (b) b.blur(); });
window.addEventListener('keyup', (e) => { if (e.key === ' ') e.preventDefault(); });
window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLSelectElement || (e.target instanceof HTMLInputElement && e.target.type === 'text')) return;
  const k = e.key.toLowerCase();
  if ([' ', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(k)) e.preventDefault();
  S.keys.add(k);
  if (e.repeat) return;
  if (k === ' ') toggleEstop();
  else if (k === 'g') setGoalTool(!S.goalTool);
  else if (k === 'escape') { if (!$('help').hidden) closeHelp(); else { setGoalTool(false); send({ t: 'cancel_goal' }); } }
  else if (k === 'h' || k === '?') $('help').hidden = !$('help').hidden;
  else if (k === 'l') cycleLayout();
  else if (k === 'm') setTarget(S.target === 'uav' ? 'ugv' : 'uav');
  else if (k >= '1' && k <= '7') setMainView(VIEW_KINDS[+k - 1]);
});
window.addEventListener('keyup', (e) => S.keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => S.keys.clear());           // never leave a key "stuck" when the tab loses focus

// Gamepad: browsers only reveal a controller after a button press on it.
const edges = new ButtonEdges();
window.addEventListener('gamepadconnected', (e) => { toast('Controller connected: ' + e.gamepad.id.slice(0, 40)); });
window.addEventListener('gamepaddisconnected', () => toast('Controller disconnected.', 'warn'));
function pollGamepadButtons(pad) {
  for (const b of edges.poll(pad)) {
    if (b === 'b') { if (!S.estop) toggleEstop(); }
    else if (b === 'a') { if (S.estop) toggleEstop(); }
    else if (b === 'y') setTarget(S.target === 'uav' ? 'ugv' : 'uav');
    else if (b === 'lb') cycleMainView(-1);
    else if (b === 'rb') cycleMainView(1);
    else if (b === 'start') cycleLayout();
    else if (b === 'back') setGoalTool(!S.goalTool);
  }
}

// Send the sticks 20 times a second, zeros included: the server's dead-man stops the vehicle if these messages ever stop arriving.
setInterval(() => {
  if (!S.connected || document.visibilityState !== 'visible') return;
  const pad = activePad();
  // Buttons are read here, not in the draw loop: a slow frame rate (software rendering, a busy laptop) must never swallow a short tap on B (e-stop).
  if (pad) pollGamepadButtons(pad);
  const sticks = readSticks(pad, S.keys, S.target, prefs.speed);
  S.source = sticks.source;
  if (S.target === 'uav') send({ t: 'uav_vel', vx: sticks.vx, vy: sticks.vy, vz: sticks.vz, wz: sticks.wz, scale: sticks.scale });
  else send({ t: 'ugv_vel', vx: sticks.vx, wz: sticks.wz, scale: sticks.scale });
}, 50);
setInterval(() => {
  const pad = activePad();
  setText('chip-pad', !pad ? 'none – press a button on it' : pad.id.replace(/\(.*?\)/g, '').trim().slice(0, 30) + (pad.mapping === 'standard' ? '' : ' (non-standard layout!)'));
  $('chip-pad').style.color = !pad ? '' : pad.mapping === 'standard' ? 'var(--good)' : 'var(--warn)';
}, 500);
setInterval(() => send({ t: 'ping' }), 5000);

// ---------------------------------------------------------------- frame loop
function resize() {
  const W = stage.clientWidth, H = stage.clientHeight;
  renderer.setSize(W, H, false);
  paneEls.forEach((el, i) => { panes[i].resize(el.clientWidth, el.clientHeight); el.classList.toggle('small', el.clientWidth < 470); });
}
new ResizeObserver(resize).observe(stage);

function easePose(cur, target, k) {
  if (!target) return cur;
  if (!cur) return { ...target };
  cur.x += (target.x - cur.x) * k; cur.y += (target.y - cur.y) * k; cur.z += ((target.z ?? 0) - cur.z) * k;
  cur.yaw += wrapAngle(target.yaw - cur.yaw) * k;
  return cur;
}

let lastT = performance.now(), fpsFrames = 0, fpsT = lastT, slowSeconds = 0;
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min((now - lastT) / 1000, 0.1); lastT = now;
  // frames-per-second readout; a hint if drawing is slow (no GPU acceleration, or four panes on a weak machine)
  fpsFrames++;
  if (now - fpsT >= 1000) {
    const fps = fpsFrames * 1000 / (now - fpsT); fpsFrames = 0; fpsT = now;
    setChip('chip-fps', `${fps.toFixed(0)} fps`, fps < 12 ? 'warn' : '');
    slowSeconds = fps < 12 ? slowSeconds + 1 : 0;
    if (slowSeconds === 6 && panes.length > 1) toast('Drawing is slow. Try the "1 view" layout (side panel) or check that the browser uses hardware acceleration.', 'warn');
  }
  const k = 1 - Math.exp(-dt * 12);

  S.smooth.uav = easePose(S.smooth.uav, S.latest.uav, k);
  S.smooth.ugv = easePose(S.smooth.ugv, S.latest.ugv, k);
  const vs = prefs.scale, su = S.smooth.uav, sg = S.smooth.ugv;
  const fp = S.follow === 'uav' ? su : S.follow === 'ugv' ? sg : null;
  if (fp) for (const p of panes) if (p.kind === 'orbit') { p.orbit.target.x += (fp.x - p.orbit.target.x) * k; p.orbit.target.y += (fp.y - p.orbit.target.y) * k; }
  if (su) { uav.visible = true; uav.position.set(su.x, su.y, su.z); uav.rotation.set(0, 0, su.yaw); uav.scale.setScalar(vs); }
  if (sg) { ugv.visible = true; ugv.position.set(sg.x, sg.y, sg.z); ugv.rotation.set(0, 0, sg.yaw); ugv.scale.setScalar(vs); }
  uavTrail.visible = ugvTrail.visible = prefs.trails;
  footprint.visible = prefs.footprint && !!su && su.z > 0.6;
  if (footprint.visible) footprint.update(su.x, su.y, su.z, su.yaw, terrain.heightAt(su.x, su.y, 0));
  dropLine.visible = !!su && su.z > 0.4;
  if (dropLine.line.visible) dropLine.update(su.x, su.y, su.z, terrain.heightAt(su.x, su.y, 0));

  // mouse -> world readout (and the hover ring while the goal tool is on)
  if (S.pointer) {
    S.hover = pickWorld(S.pointer.pane, S.pointer.el, S.pointer.x, S.pointer.y);
    if (S.hover) {
      $('hud-cursor').textContent = `${fmt(S.hover.x)}, ${fmt(S.hover.y)} · ` + (S.hover.observed ? (S.hover.walkable ? 'drivable' : 'blocked') : 'not scanned');
      if (S.goalTool) hoverRing.show(S.hover); else hoverRing.hide();
    } else hoverRing.hide();
  }

  // draw every pane into its rectangle of the one canvas
  const W = stage.clientWidth, H = stage.clientHeight;
  renderer.setScissorTest(true);
  paneEls.forEach((el, i) => {
    const pane = panes[i], x = el.offsetLeft, y = el.offsetTop, w = el.clientWidth, h = el.clientHeight;
    if (w < 2 || h < 2) return;
    renderer.setViewport(x, H - y - h, w, h); renderer.setScissor(x, H - y - h, w, h);
    pane.update({ uav: su, ugv: sg }, vs);
    // beacons: constant on-screen size for whichever camera is drawing; a vehicle's own beacon/model is hidden from inside that vehicle
    const hidden = pane.hidesVehicle();
    const sizeAt = (p) => (pane.kind === 'top' ? pane.top.half * 0.05 : pane.camera.position.distanceTo(p) * 0.03);
    for (const [beacon, veh, name, pose] of [[uavBeacon, uav, 'uav', su], [ugvBeacon, ugv, 'ugv', sg]]) {
      beacon.visible = !!pose && hidden !== name; veh.visible = !!pose && hidden !== name;
      if (pose) { const s = sizeAt(veh.position); beacon.scale.setScalar(s); beacon.position.set(pose.x, pose.y, pose.z + 0.5 * vs + 1.4 * s); }
    }
    renderer.render(scene, pane.camera);
  });
  renderer.setScissorTest(false);
  for (const [veh, pose] of [[uav, su], [ugv, sg]]) veh.visible = !!pose;
}

// ---------------------------------------------------------------- start
$('rng-speed').value = prefs.speed; $('rng-scale').value = prefs.scale;
$('val-speed').textContent = Math.round(prefs.speed * 100) + '%'; $('val-scale').textContent = '×' + prefs.scale;
$('opt-world').checked = prefs.world; $('opt-scanned').checked = prefs.scanned; scenery.visible = prefs.world; terrain.setScannedVisible(prefs.scanned);
$('opt-trails').checked = prefs.trails; $('opt-footprint').checked = prefs.footprint;
terrain.setMode(prefs.colour);
document.querySelectorAll('#seg-colour button').forEach((b) => b.classList.toggle('on', b.dataset.colour === prefs.colour));
setLayout(LAYOUTS[prefs.layout] ? prefs.layout : 'inset');
requestAnimationFrame(frame);
connect();
if (!prefs.helpSeen) $('help').hidden = false;                  // first visit: show the short how-to once
updateCoach();
window.__studio = { S, panes, terrain, scenery, coachNow, updateReach, prefs, setLayout, setMainView, setTarget, setGoalTool, pickWorld };   // handle for automated checks and debugging
