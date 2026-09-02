# Step 12: Custom Construction-Site World

**Package**: `src/emap/`
**Goal**: A world matching TerraLink's actual stated domain ("construction sites, disaster response") to test against, instead of only isolated single-feature terrains (`flat`, `bump`).
**Status**: ✅ Complete and verified - every major feature's elevation confirmed against its known, designed height, including a real bug found and fixed along the way (Ignition's `<heightmap>` geometry ignoring a link pose offset).

---

## 1. Why build our own, and where the assets came from

Extensive research (Fuel, GitHub, general web search) found no ready-made, well-licensed, Ignition-native "construction site" world. The closest hits were wrong simulator (Unity), wrong Gazebo generation (Classic), tiny unverified repos, or themed-but-unrelated worlds. `robotnik_gazebo_worlds` (the repo the user pointed at) confirmed Fuel-hosted individual props exist and are usable, but none of its six themed worlds are a construction site either, and its own top-level licensing is unclear - so the final approach avoids depending on it at all.

Every asset actually used was checked individually on **Gazebo Fuel**, each with its own confirmed license:

| Element | Model | Owner | License |
|---|---|---|---|
| Crane | `Tower crane` | OpenRobotics | CC0 |
| Excavator | `MINI_EXCAVATOR` | GoogleResearch | CC-BY 4.0 (a toy scan - see Section 3) |
| People (x3) | `Walking person` / `Standing person` | OpenRobotics | CC0 |
| Cones (x3), barrels (x2) | `Construction Cone` / `Construction Barrel` | OpenRobotics | CC0 |
| Building under construction | self-authored | us | n/a - primitive boxes, no mesh |
| Terrain (pit + mound) | self-authored | us | n/a - generated heightmap |

Fuel downloads stalled unpredictably for the two ~26MB person models (a known issue for this sandbox - see step01's docs) but NOT for smaller files or the metadata API; worked around with a resumable retry loop (`curl -C -`) rather than one long blocking attempt, same spirit as this project's other network workarounds.

## 2. Two real Gazebo/Fuel packaging bugs found and fixed

**Bug 1 - `model://` URI resolution uses the folder name, not the SDF's internal `<model name>`.** The vendored `Walking person`/`Standing person` models internally declare `<model name="person_walking">`/`"person_standing">`, and their own mesh `<uri>` tags self-reference `model://person_walking/meshes/...`. Vendoring them into folders named `walking_person`/`standing_person` (to match this project's own naming convention) broke mesh resolution entirely (`Unable to find file with URI [model://person_walking/meshes/walking.dae]`), since Ignition resolves `model://` by the folder name on `GZ_SIM_RESOURCE_PATH`, not the SDF's internal name. **Fixed** by renaming the vendored folders to match the SDF's own self-reference (`person_walking`, `person_standing`) instead of fighting it.

**Bug 2 - Collada/OBJ meshes with bare-filename texture references don't resolve from a separate `materials/textures/` folder.** The crane, cones, barrel, and excavator meshes all reference their textures by bare filename (e.g. `crane_diffuse.jpg`, no path) inside the `.dae`/`.mtl` file - Ignition's mesh loader looks for these relative to the mesh's own directory, not the `materials/textures/` folder Fuel packages them into by convention. Every affected model loaded with `Could not resolve file [...]` errors and rendered untextured. **Fixed** by copying each model's texture files directly into its `meshes/` directory alongside the mesh that references them.

## 3. A real bug in our own terrain generation: heightmap `<pose>` offsets are ignored

The first design tried to make "flat ground" line up with world z=0 by giving the heightmap's `<heightmap>` geometry's own gray-value range flat-in-the-middle (pit dipping toward black, mound rising toward white), then shifting the whole thing down with a `<pose>0 0 -1.5 0 0 0</pose>` on the link so the middle gray value would land at z=0.

Live testing immediately caught this as wrong: Gazebo's own physics debug log reported the heightfield's actual collision bounds as `min={-14.9,-14.9,-0.05} max={14.9,14.9,3.5}` - the RAW, un-offset `[0, 3.5]` range, not the intended `[-1.5, 2.0]`. The depth camera confirmed it: flying over the pit measured its depth as ~0 instead of the intended -1.5. **Ignition's `<heightmap>` geometry ignores a non-zero link-level `<pose>` Z offset entirely**, for both physics and rendering.

**Fixed** by not fighting this: the heightmap link is placed at the SDF default identity pose (matching `bump_test.world`'s own already-working `bump_terrain`, which never needed an offset because its whole range already started at 0), and "flat ground" simply lives at absolute world z=+1.5 (`PIT_DEPTH_M`) instead of z=0. Every relative height (pit depth below flat, mound height above flat) is exactly as designed either way - only the absolute datum moved. Every other model in the world (crane, excavator, people, building, the UAV's own spawn pose) is placed at that same +1.5 baseline instead of 0.

## 4. The terrain and layout

`scripts/generate_construction_heightmap.py` (same grayscale-PNG technique as `generate_bump_heightmap.py`, see that script's docstring for the underlying rationale) generates a 30m field with:
- **Excavation pit**: 6m x 4m, 1.5m deep, centered at world (-4, 2) - a smoothed-rectangle depression (rounded edges over a 0.4m soften distance, not a knife-edge), so world z there = 0.
- **Spoil mound**: a Gaussian bump (2m tall, sigma 2m) centered at world (5, -4) - the dirt piled up from digging the pit - world z at its peak = 3.5.
- Everywhere else: flat, world z = 1.5.

Scene layout (all absolute world coordinates, all at the +1.5 baseline):
- `building_frame` at (-4, -5) - just south of the pit, its own foundation dig.
- `tower_crane` at (2, 3), `mini_excavator` at (0, 5) - both near the pit/building they service.
- 3 people, 3 cones (marking the pit's perimeter), 2 barrels (staged material near the crane) - scattered around the site.

`models/building_frame/` is self-authored: 6 vertical columns (corners + long-side midpoints, 0.4m x 0.4m, 10m tall) and 3 floor slabs (3m/6m/9m, no slab at the 10m top) - a skeletal, honestly-"under construction" structure rather than a disguised finished building, built from SDF primitive boxes with no external mesh (so no licensing question). See Section 6 for why this design choice was made instead of hunting further for an open mesh.

`models/mini_excavator/model.sdf` adds `<scale>26 26 26</scale>` to both the visual and collision mesh: GoogleResearch's original is a real toy scan (measured bounding box ~0.086 x 0.167 x 0.172m from its own `.obj` vertices), not a full-scale vehicle - 26x brings its longest axis to ~4.3m, roughly a real mini excavator's footprint.

## 5. Verification

Every reading below is a live measurement against the feature's known, designed elevation - not a visual "looks right" check:

| Feature | World coords | Predicted elevation | Measured | 
|---|---|---|---|
| Pit bottom | (-4, 2) | 0.0 | -0.007, -0.014 |
| Flat ground | (-4, -1), (0, 0) | 1.5 | 1.472, 1.495 |
| Mound peak | (5, -4) | 3.5 | 3.521 |
| Building top slab | (-4, -5) | ~10.65 (slab top surface) | 10.579 |
| Building column corner | (-7.8, -7.8) | 11.5 (column top) | 11.430 |
| Excavator | (0, 5) | plausible (~4.3m scaled model) | 3.962 |

All within the same few-centimeter measurement noise this project has documented at every prior step (e.g. step07's bump peak measured 1.4978m against a true 1.5m).

**One honest limitation found, not a bug**: the tower crane's mast is thin enough (a fraction of a meter) that at this map's 0.1m resolution, nearby cells only partially overlap it and read a blended value between the mast and surrounding flat ground (~1.8-2.1m, not the mast's true height) rather than cleanly resolving it. This is a genuine, expected sensor/resolution limitation for thin vertical structures, not something to "fix" - real depth sensors and grid maps have the same limitation for any object thinner than a few grid cells.

**A real physics interaction, also not a bug**: flying too low near the building frame, the UAV was physically blocked by the underside of its first floor slab (collision geometry correctly solid) - confirming the building's collision matches its visual, and a reminder that this scene has real vertical obstacles a UGV/UAV would need to actually navigate around, not just fly over trivially.

`colcon build --packages-select emap` succeeds; all 44 unit tests unaffected (this step touched only world/model assets, no `emap/` Python code).

## Follow-ups for later steps

- The mini excavator's un-to-scale proportions (a toy scan stretched uniformly) are an accepted simplification, not something planned to be replaced - matches the "doesn't have to look very real" brief this step was scoped to.
- If a future world needs a thinner-mast crane to actually resolve cleanly, that would need either a higher map resolution or a wider mast - not addressed here since the current crane is good enough as a "there's a tall obstacle here" test case.
