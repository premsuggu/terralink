# Terrain classification pictures

What each world really looks like from above, next to how our rules classify every map cell.

## Main pictures: `<world>.png` (one per stress world)

Two panels covering the same area at the same scale:

| left: **real top-down view** | right: **segmented by our rules (v2)** |
|---|---|
| The world rendered by Gazebo itself, looking straight down. | Every 10 cm map cell coloured by the class our rules give it. |

How to read the colours on the right:

| colour | class | meaning for the planner |
|---|---|---|
| green | EASY | drive freely |
| orange | DIFFICULT | allowed, costs more |
| dark red | LETHAL | never drive here |
| grey | not observed | the UAV camera never saw it |

Which map the right panel comes from:

| world | map |
|---|---|
| W4 structure decks | the **real** map emap built during a live Gazebo mission (`src/nav/maps/w4_structure_decks_uavscan.npz`) |
| W1, W2, W3, W5 | no live scan of these worlds is saved, so the **offline scan simulator** at low noise (1 cm): the live camera model + the real emap fusion (step11 X-C1) |

### Why the real view is "cut at 5 m"

Our maps describe the world **below the UAV's flight height**: the patrol flies at 3-4.5 m, so the maps keep everything up to
5 m (decks, walkway roofs) and drop what is higher (crane jibs, the upper floors of W1's 58 m concrete tower) - see
`UAV_CEILING` in `src/nav/scripts/worldgen/worlds/*.py`. The left picture shows the same slice, so the two panels line up:

* the camera is put high above the map (about 10x the map's width: 500 m for W4, 1200 m for W1) with the narrowest lens
  Gazebo accepts (0.1 rad), so the picture is almost a map (a 5 m tall object at the map's edge shifts outwards by ~1 %, at most
  0.25 m; the ground itself does not shift);
* its near clipping plane sits at 5 m above the ground: everything higher is simply not drawn;
* shadows are off (the cut-away tower would otherwise still darken half of W1).

`topdown/<world>_full.png` is the uncut view from the 150 m camera the run videos use, with shadows. It is closer to "what a
person would see", but tall things look bigger and lean outwards (W1's tower top appears 1.6x its size), so it does **not**
line up with the map.

### Things to look for

* **Walls, containers, vehicles, barrels** are thin dark-red outlines: only their edges are LETHAL; the rules judge a step
  between neighbouring cells, so the *top* of a container is EASY-coloured (the robot can never get up there, which the step
  around it already guarantees).
* **Gravel/rough patches and slopes** (W1 corners, W3) are orange/red mixtures.
* **Roofs and decks** (W1 first floor, W4 rooms and walkway) are classified from their TOP, because that is what the UAV sees.
  What is under them is planned separately (hidden-island rule, step12).
* **W4 real map:** scattered red specks on open ground are the false obstacles discussed in step12 section 4.3 - the real scan
  was thin there.

## Comparison pictures and numbers: `comparison/`

The earlier five-panel pictures (true height | ground truth | old rule | new rule | errors), for low and high sensor noise and
for W4's real scan, plus `comparison/scores.md` with every number. The ground truth there uses the **same thresholds** as our
rules (max step 0.09 m, max slope 0.35 rad - the Jackal's measured limits, step11 X-A1) on perfect, noise-free geometry, so it
tests how well the rules survive a real scan; the thresholds themselves were tested by driving the vehicle.

| scan | old rule: agreement | new rules: agreement | new: false LETHAL | new: missed LETHAL (interior, > 1 cell away) |
|---|---|---|---|---|
| simulated, W1-W5, low / high noise | 3-15 % | 98-100 % / 88-95 % | 0-0.1 % / 1.5-3.7 % | 0-0.6 % |
| **W4 real Gazebo scan** | 58.4 % | **84.9 %** | 7.6 % | 14.1 % (\*) |

(\*) about 820 of those 874 cells lie under roof edges, which the UAV-view ground truth calls a cliff although the floor below is
drivable; see the step11 and step12 docs.

## How to regenerate

```bash
source /opt/ros/humble/setup.bash && source install/local_setup.bash
python3 src/nav/scripts/render_world_topdown.py      # the real views, ~1 min per picture (headless Gazebo)
python3 src/nav/scripts/render_classification.py     # the class maps + comparison pictures, ~10 min
```

Not covered here: the UGV camera's own ground layer (step11 sections 8-9; live much noisier than offline, step12 section 7).
