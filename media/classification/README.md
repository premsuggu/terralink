# Terrain classification: ground truth vs our rules

Pictures of how every map cell is classified, for the five stress worlds. Made by `src/nav/scripts/render_classification.py`
(~10 min; method and code: `docs/work-docs/nav/step11_terrain_classification.md`, experiment X-C1).

Each cell gets one of three classes:

| colour | class | meaning for the planner |
|---|---|---|
| green | EASY | drive freely |
| orange | DIFFICULT | allowed, costs more |
| dark red | LETHAL | never drive here |
| grey | not observed | the UAV camera never saw it |

## How to read one picture (5 panels, left to right)

1. **true height** of the world (what the UAV flies over).
2. **GROUND TRUTH class**: computed from the TRUE, complete, noise-free geometry with the Jackal's MEASURED limits (step11 X-A1:
   a 9 cm step is crossed and 10 cm blocks it; ramps up to 35 deg). **Important:** this uses the same thresholds as our rules
   (max step 0.09 m, max slope 0.35 rad). So the pictures test how well the rules SURVIVE A REAL SCAN (noise, gaps, sparse points);
   whether the thresholds themselves are right was tested separately, by driving the vehicle (X-A1).
3. **OLD rule** (what the node ran before step11) on the scanned map.
4. **NEW rules v2** (today's default) on the same scanned map.
5. **Errors of the new rules**: white = correct, yellow = EASY/DIFFICULT mix-up (only changes the cost), blue = false LETHAL (free
   ground lost), **red = MISSED LETHAL** (the dangerous error: a hazard the planner would drive into).

## Files

| file | scan |
|---|---|
| `<world>_low_noise.png` | simulated UAV scan (the live camera model + the real emap fusion), depth noise 1 cm, 7 m altitude |
| `<world>_high_noise.png` | the same, noise 3 cm + 0.001 x range^2 (up to ~7 cm) |
| `w4_structure_decks_real_scan.png` | the REAL map emap built during a live Gazebo mission (`src/nav/maps/w4_structure_decks_uavscan.npz`) |
| `scores.md` | all numbers |

## The numbers (from `scores.md`)

| world / scan | old rule: agreement | new rules: agreement | new: false LETHAL | new: MISSED LETHAL, per cell | new: MISSED LETHAL, interior, > 1 cell away |
|---|---|---|---|---|---|
| W1 house site, low / high noise | 9.5 % | **99.2 / 92.3 %** | 0.1 / 3.5 % | 3.7 / 5.0 % | 0.6 / 0.2 % |
| W2 container yard | 8.3 % | **98.1 / 93.8 %** | 0.0 / 2.4 % | 8.6 / 8.5 % | 0.0 / 0.0 % |
| W3 excavation slopes | 15.2 % | **98.8 / 88.5 %** | 0.0 / 3.7 % | 1.5 / 2.4 % | 0.2 / 0.1 % |
| W4 structure decks | 3.5 % | **99.2 / 94.8 %** | 0.0 / 1.5 % | 23.0 / 22.9 % | 0.0 / 0.0 % |
| W5 door gauntlet | 5.0 % | **100.0 / 94.4 %** | 0.0 / 3.4 % | 0.0 / 0.0 % | 0.0 / 0.0 % |
| **W4 REAL Gazebo scan** | 58.4 % | **84.9 %** | 7.6 % | 31.0 % | 14.1 % |

**What this shows**

* The OLD rule calls almost everything LETHAL on a scan (100 % false LETHAL): it used the map's height *variance* as "roughness",
  and a cell seen once from 6-7 m always has a large variance. The planner then had nowhere to drive. (On the real scan the old rule
  looks better, 58 %, only because that scan's variance was small - it is still 39 % false LETHAL.)
* The NEW rules agree with the ground truth on 92-100 % of the cells on simulated scans, and 85 % on the real one.
* **Two columns for missed LETHAL, and why.** "Per cell" is strict. Walls in the truth are often ONE cell (10 cm) thin, so a wall
  predicted one cell to the side counts as a miss *and* a false LETHAL, although the planner (which keeps the robot's radius away
  from every LETHAL cell) would avoid it anyway. And at the RIM of the observed area (e.g. the site's boundary fence at the edge of
  the scan) the far side of a wall was never measured, so no rule can classify it (step11, "the rim limitation"). The last column
  forgives a one-cell offset and counts only cells whose whole 5 x 5 neighbourhood was observed: on simulated scans 0-0.6 %.
* **The real W4 scan, 14.1 %: read it carefully.** About 820 of its 874 "missed" cells are under the EDGES OF ROOFS: the scaffold
  walkway roof (y ~ -18) and the deck edge over the west door. This ground truth is the UAV's view, which treats a roof edge 3.2-3.5 m
  up as a cliff; the real camera, looking at an angle, saw the floor underneath - and the UGV really can drive there (goal D is under
  that walkway). So most of those are not hazards the robot would hit. The real scan's 7.6 % false LETHAL is the bigger practical
  problem: dotted specks where the scan was thin (step12, section 4.3).
* Noise costs mainly agreement through EASY <-> DIFFICULT mix-ups (yellow), not safety: noise compensation (step11 section 4b) keeps
  noise from turning flat ground LETHAL.

## Not covered here

The UGV camera's own ground layer (step11 sections 8-9). Live it was much noisier than offline (step12 section 7: a third of its
LETHAL cells were false, flying pixels at edges).
