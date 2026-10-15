"""Self-contained HTML 3D view of a world (`media/worlds/<w>_world.html`).

A plain orbit view of the ACTUAL world geometry (the same triangle meshes the Gazebo
world is built from - not a point cloud), lit and coloured with the pack's palette
colours, over a ground plane. No panels or buttons. The meshes are embedded in the
page, so the file opens by double-click; only three.js (pinned, same version as
`media/3d_maps/viewer.html`) comes from a CDN.

Embedding: all items are merged into one indexed mesh. Positions are quantised to
uint16 inside the world's bounding box (< 1 mm per 65 m), triangle indices are
uint32, and each triangle carries one uint8 RGB colour (flat paint look); all base64.

Controls: left-drag orbit, right-drag pan, wheel zoom. Z is up, world coordinates.
"""
from __future__ import annotations

import base64

import numpy as np

_PAGE = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>{title}</title>
<style>
  html, body {{ margin: 0; height: 100%; overflow: hidden; background: #cfd8e3; font-family: sans-serif; }}
  #hint {{ position: fixed; left: 12px; bottom: 10px; color: #234; font-size: 13px; opacity: .75; pointer-events: none; }}
</style></head>
<body>
<div id="hint">{title} &middot; drag: orbit &middot; right-drag: pan &middot; wheel: zoom</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const NV = {nv}, NT = {nt}, LO = {lo}, SPAN = {span};
const GROUND = {{ z: {gz}, half: {gh}, rgb: {grgb} }};
const bytes = s => Uint8Array.from(atob(s), c => c.charCodeAt(0));
const q = new Uint16Array(bytes("{pos}").buffer);
const idx = new Uint32Array(bytes("{idx}").buffer);
const rgb = bytes("{col}");

// Non-indexed copy so every triangle can have its own flat colour.
const pos = new Float32Array(9 * NT), col = new Float32Array(9 * NT);
for (let t = 0; t < NT; t++) for (let k = 0; k < 3; k++) {{
  const v = idx[3 * t + k];
  for (let a = 0; a < 3; a++) {{
    pos[9 * t + 3 * k + a] = LO[a] + q[3 * v + a] / 65535 * SPAN[a];
    col[9 * t + 3 * k + a] = rgb[3 * t + a] / 255;
  }}
}}

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xcfd8e3);
const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 5000);
camera.up.set(0, 0, 1);                       // world Z is up
const renderer = new THREE.WebGLRenderer({{ antialias: true }});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);

const g = new THREE.BufferGeometry();
g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
g.setAttribute('color', new THREE.BufferAttribute(col, 3));
g.computeVertexNormals();                     // non-indexed => flat normals
scene.add(new THREE.Mesh(g, new THREE.MeshLambertMaterial({{ vertexColors: true, side: THREE.DoubleSide }})));

const ground = new THREE.Mesh(new THREE.PlaneGeometry(2 * GROUND.half, 2 * GROUND.half),
  new THREE.MeshLambertMaterial({{ color: new THREE.Color(...GROUND.rgb) }}));
ground.position.set({gcx}, {gcy}, GROUND.z - 0.02);   // just under the items so coplanar faces do not z-fight
scene.add(ground);

scene.add(new THREE.HemisphereLight(0xffffff, 0x778899, 0.75));
const sun = new THREE.DirectionalLight(0xffffff, 0.65);
sun.position.set(-40, -30, 80);
scene.add(sun);

const centre = new THREE.Vector3(LO[0] + SPAN[0] / 2, LO[1] + SPAN[1] / 2, LO[2] + SPAN[2] / 2);
const reach = Math.max(SPAN[0], SPAN[1]);
camera.position.set(centre.x + reach * 0.55, centre.y - reach * 0.75, centre.z + reach * 0.65);
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.target.copy(centre);
controls.update();

addEventListener('resize', () => {{
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight);
}});
(function loop() {{ requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); }})();
</script></body></html>
"""


def write_world_html(scene, title: str, path) -> int:
    """Write the page for a `Scene` (items' verts/tris/colours + ground); returns the triangle count."""
    verts, tris, cols, off = [], [], [], 0
    for it in scene.items:
        verts.append(np.asarray(it.verts, np.float64))
        tris.append(np.asarray(it.tris, np.int64) + off)
        cols.append(np.asarray(it.colours, np.uint8))
        off += len(it.verts)
    V, T, C = np.concatenate(verts), np.concatenate(tris), np.concatenate(cols)
    lo, hi = V.min(0), V.max(0)
    span = np.maximum(hi - lo, 1e-6)
    q = np.round((V - lo) / span * 65535).astype("<u2")
    b64 = lambda a: base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()      # noqa: E731
    gh = float(max(span[0], span[1])) / 2 + 6          # ground plane: the items' footprint plus a margin
    page = _PAGE.format(
        title=title, nv=len(V), nt=len(T),
        lo="[" + ",".join(f"{v:.4f}" for v in lo) + "]", span="[" + ",".join(f"{v:.4f}" for v in span) + "]",
        gz=scene.ground_z, gh=gh, grgb="[" + ",".join(f"{c:.3f}" for c in scene.ground_rgb) + "]", gcx=f"{lo[0] + span[0] / 2:.3f}", gcy=f"{lo[1] + span[1] / 2:.3f}",
        pos=b64(q), idx=b64(T.astype("<u4")), col=b64(C))
    with open(path, "w") as f:
        f.write(page)
    return len(T)
