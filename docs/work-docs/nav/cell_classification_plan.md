# Cell classification - plan (PROPOSAL, awaiting approval)

Written 2026-10-26. Nothing in this plan has been implemented. It answers "how do we make the cell classification of the core
pipeline good enough?" and follows the project rules: propose -> approval -> implement step by step with unit tests; reuse before
building. Context: `TODO.md` section 2 (research summary), `docs/work-docs/emap/step07_traversability.md` (the current classifier).

## 1. What the classifier does today, and where it fails
`emap/traversability.py: compute_traversability` gives every cell one of three scores (LETHAL 0.0, DIFFICULT 0.3, EASY 1.0) from three
numbers per cell, and `nav/walkability.py` turns it into the planner's walkable mask (observed and not LETHAL):
* slope `|grad(elevation)|` > `max_slope` 0.35 (about 20 deg) -> lethal, > half of it -> difficult
* step height (max - min in the 3x3 neighbourhood) > `max_step` 0.15 m -> lethal, > half -> difficult
* "roughness" = the map's **variance layer** > `max_roughness` 0.05 -> lethal

The measured weaknesses (each one was hit in a real run, see `step09`):
| # | Weakness | Evidence |
|---|---|---|
| W-1 | **Variance is measurement uncertainty, not roughness.** It is 0.01 x range^2 per observation: a flat cell seen once from 6 m reads 0.36 > 0.05 = lethal | stress-world scans needed `max_roughness:=0.5` (and flying higher/sparser gave 57 % "lethal" ground) |
| W-2 | **No confidence.** "Seen once" and "seen 30 times" look the same; unknown is a separate flag, but an uncertain cell cannot be "walkable with low confidence" | frontier logic has to guess; the lethal ring of a roof edge is as certain as a wall |
| W-3 | **Fixed thresholds from a textbook, not from the vehicle.** 0.35 slope / 0.15 m step were never measured on the robot; the Jackal climbs an 8 cm curb and a 20 deg ramp in simulation (`step10`), so the numbers may be too strict or too loose | no experiment links them to what the UGV can really do |
| W-4 | **No ground type.** Gravel, mud, grass, water, hard floor and a painted line are the same cell if the height is the same | needs a second information source (images) |
| W-5 | **Negative obstacles** (holes, trench edges, ramps' drop-offs) only appear through the 3x3 step test; a hole narrower than 2 cells or seen from the side can vanish | W3's pit design |
| W-6 | **Height alone decides "roof or floor"** (deck vs ground) - fixed by the island logic of step09, not by classification | `nav/overhead.py` |
| W-7 | **Three tiers only**: the planner and Nav2 cannot rank two EASY cells (a smooth floor and a bumpy-but-flat field cost the same) | route choice ignores comfort / risk |

## 2. Goal and non-goals
* **Goal:** a per-cell traversability *cost* with a *confidence*, whose parameters come from measurements of the real vehicle model, and
  an optional *semantic class* (ground type) that can raise or lower the cost - feeding the existing A* planner and Nav2 costmap
  unchanged (the planner only needs walkable mask + cost).
* **Non-goals:** moving objects (other team); real-hardware tuning; replacing A* or Nav2.
* **Environment facts:** the machine has an NVIDIA RTX 3050 Laptop GPU, **4 GB**, driver 610.60 (via WSL2). `torch` is **not installed**;
  CuPy not confirmed. Gazebo runs on software GL (the GPU is not used by the simulator), so the whole 4 GB is available for models. The
  geometric part must keep working on the CPU alone (CI and unit tests).

## 3. Reuse survey (research of 2026-10-26; claims are from papers/READMEs, not re-measured by us)
| Option | What it gives | Fit |
|---|---|---|
| `elevation_mapping_cupy` (leggedrobotics) + ROS 2 Humble ports (iit-DLSLab, amilearning) | GPU elevation map with fused semantic layers, traversability filters (learned) | strong candidate for a *comparison* (phase E); CUDA 12 / large dependency; replaces our `emap` if adopted |
| Wild Visual Navigation (ETH, RSS 2023, extended 2024) | traversability from camera + the robot's own driving, learned online in < 5 min (DINO features) | good for "soft vs hard" without labels; needs torch; real-robot oriented |
| GANav / SegFormer-type segmenters trained on RUGD, RELLIS-3D, GOOSE, GOOSE-Ex (excavator data) | per-pixel ground class -> cost table | mIoU ~46-55 on real data; **no construction-specific pretrained weights found**; models trained on real photos will not transfer to Gazebo renders without fine-tuning |
| URA\*, "Seeing Where to Deploy" (2026) | UAV RGB -> semantic traversability -> UGV planning | same architecture as ours; shows the shape of phase C/D, code availability not checked |
| Geometric filters (ETH `traversability_estimation`, `grid_map_filters`) | slope/step/roughness with explicit definitions | what we have; use their *definitions* of roughness as reference for phase A |

## 4. Proposed phases (each ends with tests, a documented result and your go-ahead for the next)
### Phase A - fix the geometry (no new dependency, CPU only) - recommended first
* **A1 Roughness from the elevation, not from the variance.** roughness = std of the detrended elevation in a window (e.g. 5 x 5 cells =
  0.5 m: height minus the best-fit plane); the variance layer stays what it is, an uncertainty.
* **A2 Confidence layer.** From the variance (and an observation count if `emap` can expose it cheaply): `conf = 1 / (1 + var / var_ref)`.
  Low-confidence cells are *not lethal*: they are walkable-with-penalty, or treated like frontier cells by the explorer.
* **A3 Cost, not three tiers.** Keep `{LETHAL, DIFFICULT, EASY}` as a derived view (the planner, the PLY maps and 408 existing tests use it), add a
  continuous `cost` layer in [0, 1] from slope, step, roughness and confidence; Nav2's costmap can use it later.
* **A4 Negative obstacles.** Drop-off detection with a larger window and direction (an edge is only a hazard when the robot is on the high side).
* **A5 Parameters from the vehicle.** Experiment **X-A1**: drive the Jackal model at ramps of 10-35 deg, curbs of 4-16 cm, ruts and bumps in a
  test world (extension of `tests/nav/integration/ugv_terrain_test.world`); record success, tilt (roll/pitch from the pose) and slip; derive
  `max_slope`, `max_step` and a "difficult" band from the data instead of 0.35 / 0.15.
* **Evidence:** unit tests on synthetic terrain (ramp, step, pit, rubble, noisy flat, sparse observation); then **X-C1** on W1-W5: per cell,
  agreement of the classifier with the ground-truth map's classes (`<w>_groundtruth.npz`) for the true UAV scan - target: the default
  `max_roughness` works for a real scan (no 0.5 override) and the fraction of falsely-lethal flat cells < 1 %.
* **Risk:** low. Needs the `emap` side to expose confidence/observation count; a small change in `elevation_mapping_node.py`.
* **Estimate:** 2-3 working days incl. X-A1.

### Phase B - ground types in the simulator (ground truth for semantics)
Gazebo Fortress cannot simulate soft ground or sinkage. What it can do honestly: **friction patches + visual textures + a class truth map**
(gravel, grass, mud = low friction, water = a no-go region with a visual), plus the **segmentation camera** (a Gazebo sensor type that outputs
a per-pixel label image; availability in this Fortress install to be verified) so every camera frame has a ground-truth class image.
Deliverable: a terrain-test world W6 with 4-5 ground types and its class truth map. **Limitation to write down:** results here test the
*pipeline*; they say nothing about real soil physics.
* **Estimate:** 1-2 days. **Risk:** medium (segmentation camera availability; textures vs the world generator).

### Phase C - a semantic layer (needs approval of new dependencies: PyTorch with CUDA)
Staged so that the expensive question is answered last:
1. **C1 oracle semantics:** feed the *ground-truth* class image (segmentation camera) through the fusion (phase D) - proves the data path
   and the planner's use of classes with zero model risk.
2. **C2 a real model on the 4 GB GPU:** candidates, smallest first: a SegFormer-B0/B1-class segmenter fine-tuned on sim images labelled by the
   segmentation camera (and RUGD/GOOSE-Ex for realism); or DINOv2-small features + a tiny head (WVN-style, labels from driving). Both fit in
   4 GB for inference at 320-640 px. Decision by experiment **X-C3**: mIoU on held-out sim images, latency per frame, GPU memory.
3. **C3 (optional, later) self-supervised from driving** (WVN idea): the UGV labels cells it drove over successfully; needs C2's backbone.
* **New dependencies for C (not installed, need your approval):** `torch` + `torchvision` (CUDA build, ~2-3 GB download), optionally `transformers`/`timm`.
* **Estimate:** C1 2 days, C2 4-6 days. **Risk:** high for sim-to-real, medium for the pipeline.

### Phase D - fusion and use
Fuse geometry cost and class cost per cell with confidence (class costs from a small table, e.g. hard floor 0, gravel 0.2, grass 0.3, mud 0.7,
water lethal); publish as the existing `traversability` layer plus `cost`/`class`/`confidence` layers; A* uses cost for its step cost
(it already has `cost_multiplier` machinery); Nav2's costmap reads the same layer. Experiment **X-C2:** the same goals in W6 and W1-W5 with
geometry only vs geometry+semantics: success, path cost, time, and "did it avoid the mud/water".

### Phase E - compare with `elevation_mapping_cupy` (decision, not a build)
Run its ROS 2 Humble port on the same scans (needs CUDA 12 + CuPy; to be checked against the 4 GB card) and decide: keep our `emap` + new
classifier, or adopt theirs. Output: a one-page comparison (accuracy on X-C1, speed, effort to integrate).

## 5. Order and gates
A (no approval of dependencies needed) -> your review -> B -> your review -> C1 -> your review (and approval to install torch) -> C2 -> D -> E.
Phase A alone already addresses W-1, W-2, W-3, W-5 and W-7; B-D address W-4.

## 6. Decisions I need from you
1. Start with Phase A as the only thing now? (recommended; no installs, CPU only)
2. Is a **simulation-only** proof of the semantic layer (phase B/C: friction patches + oracle labels first) acceptable, or do you want real-photo
   datasets (RUGD / RELLIS / GOOSE-Ex) in the loop from the start? (the latter needs downloads of several GB and the torch install early)
3. May I install PyTorch (CUDA) when phase C starts, and is a ~3 GB download fine? Alternatives: ONNX Runtime only (smaller, inference only).
4. Which matters more for the use cases you have in mind: *ground type* (gravel/mud/grass/water) or *geometry quality* (rough ground, slopes, holes)?
   That decides whether C is worth doing before E.
