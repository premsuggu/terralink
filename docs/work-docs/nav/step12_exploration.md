# Step 12 - Exploration: testing existing solutions against ours (EX-01..EX-03)

**Date:** 2026-10-27 / 28. **Status:** offline comparison done; island-rule bug fixed; freespace planning under roofs ADOPTED
(user approval 2026-10-28) as `island_strategy:=freespace` (default still `viewpoints`); ground-layer confidence gate added; two live
W4 room-B runs - **neither reached the goal**: the planner now gets to the room next to B, and the camera's false obstacles (flying
pixels at door edges) close B's 1.0 m door. See section 7.

**Reading order:** 1 (the problem) -> 2 (what was tested and how) -> 3 (a bug found on the way) -> 4 (results) -> 5 (what it
means) -> 6 (proposal and open decisions).

---

## 1. The problem

The UAV sees only the roof of a building. For a goal *inside* (W4 rooms A, B, C, F) the UGV must discover doors and corridors
with its own sensors. The only live W4 success so far (goal B, 2026-10-27, ground layer on) drove **366 m for a 17 m goal** with
89 "your plan is blocked" re-plans and 20 target switches (step11 doc, section 10). The user's direction: test existing
solutions first and adopt one if it is good, instead of building everything ourselves.

## 2. What was tested, and how

### 2.1 Candidates

| Candidate | What it is | Tested? |
|---|---|---|
| **Ours** | `nav.global_planner` + `nav.exploration`: A* on the map; if no route, go to the best *frontier viewpoint* (edge of unknown space, scored by reach cost + distance to goal - gain); route memory, blacklist, stuck detector | yes |
| **Freespace assumption** (D* Lite, Koenig & Likhachev 2002; Nav2 `allow_unknown`) | Plan straight to the goal **as if unobserved cells were free**; re-plan as the sensors reveal walls. `unknown_cost` = cost of an unknown cell relative to a known free one: 1 = pure D* Lite; ~3 = Nav2 Smac2D (default `cost_travel_multiplier` 2: `1 + 2*255/252`); ~5 = Nav2 NavFn (unknown = 253 vs free 50) | yes, costs 1, 2, 5 |
| **explore_lite** (`m-explore-ros2`, ROS 2 port of Horner's explore_lite) | Frontier clusters reachable from the robot, cost = `3 * d * res - 1 * size * res`, drive to the best cluster's **centroid** with Nav2, blacklist on Nav2 abort or 30 s without progress. Goal-free: here it explores until a route over *observed* ground to the goal exists, then drives it | yes (decision rule ported, verified against `explore.cpp`: `target_position = frontier->centroid`; Humble blacklists every aborted goal) |
| TARE (CMU) | Has a ROS 2 Humble/Jazzy branch, but runs inside CMU's own stack (its local planner, terrain analysis, waypoint following) on registered 3D lidar scans, and is goal-free coverage exploration | **no** - would replace Nav2 + our map, needs a 3D lidar; our task is reaching a goal, not covering the site |
| GBPlanner2, FAEL | ROS 1 / 3D lidar, coverage exploration | **no**, same reasons |

**Important:** the candidates were tested as *ports of their decision rules* (`tests/nav/explore_policies.py`, a few dozen lines
each), not by running the packages. That isolates the strategy - same map, same sensor, same robot - which is what we needed to
choose. A live run of the real package is only worth doing for the winner.

### 2.2 Method: the offline simulator

`tests/nav/explore_sim.py` (step09) drives a simulated UGV against a ground-truth map: plan -> drive 2 m -> lidar scan (the live
`floor_map_node` code) -> re-plan. A live W4 mission costs ~40 min of wall time; one simulated goal costs 5-40 s. Changes for
this step:

| Change | Why |
|---|---|
| `policy=` | any planner with `GoalIslandPlanner.plan`'s signature can be plugged in; every candidate is wrapped in our hidden-island rule, so all see the same map (the roof hidden as "unknown") |
| `overhead_on_switch_only=True` | the ~10 s "new goal" cost (stop, turn, accelerate; measured live) is charged only when the target really moves (> 1 m); a re-plan toward the same goal is not a new goal. A *blocked* attempt still costs the full 10 s (Nav2 tries and recovers) - without that, the simulator ended runs before our 18 s stuck detector could fire |
| `ground_layer=True` | models the UGV camera ground layer (`ground_layer:=true`): cells in a +-30 deg, 8 m cone, line of sight, take their true class (offline the v2 classifier on the UGV camera was 100 % correct on W4, step11 X-C4) and win over the UAV map via the live `combine_ground` |

Driver: `src/nav/scripts/eval_exploration.py` (any world: start and goals come from the world's `MARKERS`).

```bash
python3 src/nav/scripts/eval_exploration.py --goals B --maps uavscan                 # all policies, W4 goal B, real scan
python3 src/nav/scripts/eval_exploration.py --world w5_door_gauntlet --maps uavview --no-ground --policies ours island_freespace
```

Metrics per goal: reached, metres driven, plans, **target switches** (final point moved > 2 m), simulated time.
Maps: `uavview` = ideal UAV map (rendered from the world), `uavscan` = the REAL emap scan of the live W4 mission (W4 only).

**What the simulator does not model** (so a live run is still the final check): Nav2's local planner and costmap (it will not
drive a gap the global map calls free if its footprint does not fit - the "0.5-0.6 m door" problem), the `plan_blocked` event
re-plan, wheel slip, camera noise.

## 3. A bug found on the way: the ground layer switched off the hidden-island rule

The first comparison with the ground layer on said **"goal on ordinary ground"** for goal B - a goal deep under the roof.

*Why:* `planner_node._map_callback` merged the ground layer into the UAV map (`combine_ground`) **before** the island rule
(`nav.overhead.goal_island_zone`) looked at it. Wherever the camera had looked in under the deck, the deck's 3.5 m height was
replaced by the floor's 0 m. The walkable mask ignores height, so those cells joined the deck plateau to the ground outside: the
roof was no longer an "island", its flat top was trusted, and A* planned straight across it - through walls nobody had seen.

*Evidence from the live W4 run (2026-10-27):* of 31 logged planner decisions, **29 said "goal on ordinary ground"**. Likely
mechanism for the 366 m and the 89 blocked re-plans: with the island rule off, the deck top counted as open floor, so the planner
was effectively making freespace-style routes straight across the building - but the 360 deg lidar floor map never reaches those
cells (`compose_with_floor` only fills cells the UAV never observed or that lie in a hidden zone), so walls were learned only
through the +-30 deg camera cone, one `plan_blocked` at a time.

**Warning - the fix alone may make the live result worse.** The one live success happened WITH the bug. With the fix and today's
default strategy (viewpoints), the simulator does not reach B on the real scan (4.1). So the fix should go live together with the
freespace option (section 6), or knowingly on its own. (The simulator did not reproduce the bugged live run either - our planner
stopped after 16 m there - one more reason to read the offline numbers as a ranking of strategies, not a forecast of live metres.)

*Fix* (`src/nav/nav/planner_node.py`, `_map_callback`): the island planner gets the **UAV layers alone**; the ground layer is
merged afterwards, inside `make_ctx -> build_masks`, which already calls `combine_ground`. The simulator does the same.
Tests: `tests/nav/test_overhead.py::test_ground_layer_merged_first_would_hide_the_island` (documents the failure mode) and
`::test_island_planner_with_ground_layer_hides_the_island_and_still_uses_the_ground_layer` (the fixed path: island hidden, and
what the camera saw under the roof is still used).

## 4. Results

All numbers: start S of each world, re-plan every 2 m, 0.5 m/s, 10 s per new target and per blocked attempt. "fs" =
freespace. Raw output of every run is reproducible with the commands in 2.2.

### 4.1 W4, the REAL UAV scan, ground layer on (the configuration of the only live success)

| goal | ours | fs, cost 1 | fs, cost 2 | NavFn-like (cost 5) | explore_lite |
|---|---|---|---|---|---|
| A far room | 104 m, 16 switches | **91 m, 0** | 65 m, 0 | 66 m, 0 | 68 m, 11 |
| B dead-end room (oracle 76 m) | **not reached** (518 m, 76) | **99 m, 0** | 204 m, 0 | 183 m, 0 | 284 m, 41 |
| C sealed (infeasible) | wanders 492 m | **gives up at 113 m** | gives up 113 m | gives up 92 m | wanders 357 m |
| D under walkway | 29 m, 3 | 28 m, 0 | 28 m, 0 | 28 m, 0 | 32 m, 8 |
| E outside | 63 m, 0 | 65 m, 0 | 65 m, 0 | 65 m, 0 | 73 m, 10 |
| F SE room | 70 m, 11 | 66 m, 0 | 41 m, 0 | 41 m, 0 | 349 m, 46 |
| **sum A+B+F** | B fails | **256 m** | 310 m | 290 m | 701 m |

Simulated time for B: freespace 207 s (switch-only overhead) to 677 s (10 s on every one of its 48 plans - the pessimistic bound;
live, a re-plan toward the same goal republishes `/goal_pose` and Nav2 preempts without a full stop, so the truth is in between).

### 4.2 W4, the ideal UAV map (`uavview`): same picture

| goal | ours | fs, cost 1 | fs, cost 2 / NavFn-like | explore_lite |
|---|---|---|---|---|
| A | 90 m, 16 switches, 460 s | **89 m, 0, 188 s** | 65 m, 0 | **not reached** (stalls at 20 m) |
| B | 116 m, 23, 651 s | **99 m, 0, 207 s** | 204 / 172 m | not reached |
| C (sealed) | wanders 281 m | gives up at 116 m | gives up | gives up at 20 m (wrongly: no frontier left) |
| F | 73 m, 11, 315 s | 64 m, 0, 138 s | 41 m | not reached |
| D, E | 28 / 62 m | 29 / 63 m | 29 / 63 m | 29 / 63 m |

explore_lite stalls because, once the robot is inside, the whole unknown interior is ONE connected frontier whose **centroid**
(its target) lies in the middle of the building; driving there never brings the robot closer to the nearest frontier cell, so after
30 s the only frontier is blacklisted and it stops - a known weakness of centroid targets, not a porting artefact (verified in its
source).

### 4.3 W4 real scan WITHOUT the ground layer (today's live default)

Freespace (any variant) and ours both **fail A and B**: a dotted lattice of false "lethal" cells in the real scan, in front of the
west entrance (flat floor in truth: 38 lethal + 513 difficult cells where the scan was thin), becomes a wall once inflated by the
robot radius. Without the ground layer nothing can correct a UAV *lethal* cell outside the hidden island - the floor lidar only
fills cells the UAV never observed (`compose_with_floor`). D, E, F are reached by all. **Conclusion: on a real scan, any
strategy needs the ground layer (or another way for the UGV to correct false UAV obstacles).**

### 4.4 Regression on the other worlds (ideal maps, ground layer off)

"island fs" = freespace only when the goal or the robot is under a roof (`IslandFreespacePlanner`), ours everywhere else.

| world | ours | fs everywhere | island fs |
|---|---|---|---|
| W1 house site (A feasible; B inside a closed building and C outside a double fence are infeasible) | A 106 m; B not concluded after 286 m (49 switches) | A 108 m; **gives up** on B at 241 m and on C at once | A 106 m (= ours; B, C rows did not finish within the 58-min wall-clock cap of this batch) |
| W2 container yard (A, B, C, D, F feasible; E sealed, G 0.5 m lane) | A, B, D, F reached; C (0.8 m lane) not; wanders 194 m on E and G | **all feasible reached** (C 48 m); gives up on E and G at once | = ours |
| W3 slopes (B on a steep mound = infeasible) | A, C-F reached | same (+0-1 m) | = ours |
| W5 door gauntlet | doors 1-2 passed, 3+ not | same | = ours |

W5 note: doors 3+ fail for every strategy alike, so this does not separate them. Hypothesis (NOT verified): the simulator blocks
the robot at exactly the planner's inflation radius (0.27 m), so a one-cell difference between the UAV map and the truth closes a
door. The live W5 runs (step10) remain the reference for door widths.

## 5. What it means

* **Why freespace wins.** Our viewpoint exploration asks "which edge of the unknown is best to *look at*?" and walks the facade in
  short legs, switching target as scores change (16-76 switches). The freespace rule asks a simpler question: "what is the
  shortest way to the goal if everything I have not seen is open?" It drives that way; when the lidar reveals a wall the route
  bends around it - the target never changes, so there is nothing to dither between. When even that optimistic question has no
  answer (every wall of a sealed room is known), the goal is provably unreachable on the current map and it stops.
* **Why cost 1.** A higher cost for unknown cells (Nav2's NavFn ~5, Smac2D ~3) makes the robot prefer known ground: shorter on A
  and F (it goes round the outside to a door), but twice as long on B (it tours the building before entering). Cost 1 has the
  best total and the best worst case on W4. Single-world tuning: noted as a risk, see 6.
* **What can be reused as-is.** The algorithm (D* Lite's freespace assumption), not a package we found. Nav2 itself plans with
  `allow_unknown`, but only with its own unknown cost (NavFn ~5; Smac2D `1 + m*255/252`, i.e. ~3 with the default multiplier 2,
  and ~1 only with `cost_travel_multiplier` ~0, which also switches off its keep-away-from-walls cost), and handing global
  planning to Nav2 would drop the UAV map and the hidden-island rule. explore_lite and TARE are coverage explorers (no goal);
  explore_lite lost clearly here.
* **The bug fix and the strategy belong together** (section 3 warning): the fix makes the island rule work again, and the
  freespace rule is what makes a working island rule efficient.
* **On the real scan the ground layer is required** for every strategy (4.3).

## 6. Proposal and open decisions

**Proposal (not implemented; awaiting approval):**

1. Move `FreespacePlanner` from `tests/nav/explore_policies.py` into `src/nav/nav/freespace.py` (unknown cost 1, same inflation and
   start-footprint handling as our A*), and keep our `StuckTracker` penalty zones in its cost: freespace has no blacklist, and live
   Nav2 refuses gaps the map calls free (the 0.5-0.6 m door), which the simulator cannot show.
2. A `planner_node` parameter / `tunnel_demo.launch.py` argument `island_strategy:=viewpoints|freespace` (default `viewpoints` =
   today): with `freespace`, goals under a roof are planned with the freespace rule; everything else is unchanged.
3. Unit tests for each piece; re-run this comparison; then ONE live W4 goal-B mission (`ground_layer:=true
   island_strategy:=freespace`, with the section-3 fix), compared with the 366 m / 89 blocked re-plans of 2026-10-27. Success =
   reached, < 150 m. The sensor freespace relies on inside the roof - the 360 deg lidar floor map - does run live in that
   configuration (`floor_map_node` starts with `overhead_islands:=true`; it was publishing `/ugv_floor_map` in the 2026-10-27 run).
4. Only if that works: decide whether freespace should also be used outside roofs (it was better on W2, but a UAV gap in the open
   can be a pit the 2D lidar cannot see).

**Open decisions for the user:**
* The ground layer: on a real scan every strategy needs it (4.3). Turn it on for missions (it is off by default today), or let the
  floor lidar clear false UAV obstacles at ground level (risk: a 2D lidar cannot see low steps or pits)?
* Whether to also run the real explore_lite package live (needs a source build in a separate workspace; `slam_toolbox` is already
  installed). Given 4.1-4.2 this is not recommended.

**Risks:** tuned on one hidden-interior world (W4); the simulator has no Nav2 local planner; one live run is an example, not a
statistic.


## 7. Adoption and the first live runs (2026-10-28)

### 7.1 What was built

| Piece | File | What it does |
|---|---|---|
| Freespace planner | `src/nav/nav/freespace.py` (`FreespacePlanner`) | A* to the goal with unobserved cells passable at cost x1, known obstacles inflated by the full 0.27 m (robot radius + margin), A*'s own start handling (footprint + self-pollution blob), keep-away-from-walls and DIFFICULT costs, and the SAME `StuckTracker` as the GlobalPlanner (penalises the stretch ahead when the robot makes no progress - freespace has no blacklist). A route through unknown space is flagged tentative (`has_frontier_segments`) so the follower keeps re-planning. No route even through unknown space = "walled off" (the sealed-room answer). |
| Dispatcher | `freespace.py` (`IslandStrategyPlanner`) | Freespace while `GoalIslandPlanner` has hidden an island (goal or robot under a roof), the normal GlobalPlanner otherwise. |
| Switch | `planner_node` param / `tunnel_demo.launch.py` arg `island_strategy:=viewpoints|freespace` | default `viewpoints` (unchanged behaviour); an unknown value is rejected |
| Ground-layer gate | `src/nav/nav/ground_layer.py` `MIN_CONFIDENCE = 0.15` | a camera ground-layer cell overrides the UAV map only if `ground_confidence >= 0.15` (7.3) |
| Faster videos | `scripts/make_run_video.py --speed`, `run_world_mission.sh` `SPEED` (default 4x), `VIDEO` | a 900 s run renders as a ~4 min video; the clock on screen stays in simulated seconds |

Tests: `tests/nav/test_freespace.py` (7: through unknown, uses the door not the wall, gives up when walled in, observed route not
tentative, own footprint free, the stuck penalty moves the route to the other door, the dispatcher), node tests for the parameter
(`test_planner_node_astar.py`), the gate (`test_ground_layer.py`). 477 nav tests pass. The production planner reproduces the
evaluated one offline (`eval_exploration.py --policies adopted --maps uavscan`): A 90 m, **B 98 m**, C given up at 116 m, D 29, E 63,
F 62 m, 0 target switches on A, B, C, E, F.

### 7.2 Live run 1 (`media/runs/w4_structure_decks_mission_freespace_run1_failed.mp4`)

Real pipeline (UAV scan, emap, `ground_layer:=true island_strategy:=freespace`), W4 start -> room B, 900 sim s cap. The island rule
now works ("goal is on an elevated island" in every decision) and the planner made freespace routes - but the robot drove only
**71 m** and spent most of the run stuck in the south-west room: the planner said "walled off" or routed into doors Nav2 could not
use. Snapshots of the live maps replayed offline showed why: **a third of the camera ground layer's LETHAL cells were false**
(1235 of 3795): 1-3 cell specks on open floor, ~12 cm BELOW it, median confidence 0.06 (real obstacles 0.22). The ground layer wins
over every other map, so the specks, inflated by the robot radius, sealed the doors. This contradicts the offline X-C4 result (100 %
correct on W4): the offline camera model had no such artefacts.

### 7.3 The confidence gate

Ground-layer cells with `ground_confidence < 0.15` no longer override the UAV map (they fall back to it and, under a roof, to the
lidar floor map). On run 1's snapshot:

| min confidence | ground cells kept | LETHAL | FALSE LETHAL | missed LETHAL | route from robot / from start |
|---|---|---|---|---|---|
| 0 (before) | 22520 | 3795 | 1235 | 336 | no / no |
| 0.10 | 20004 | 2276 | 440 | 309 | yes / yes |
| **0.15** | **18680** | **1825** | **272** | **309** | **yes / yes** |
| 0.30 | 15858 | 1166 | 95 | 308 | yes / yes |

Caveats: tuned on the same world and goal it was then tested on; the offline exploration simulator has no confidence layer, so the
step12 offline numbers do not cover it; a real low obstacle seen only once (below the lidar's height) is ignored until seen with
confidence. It treats a symptom; the cause is 7.5.

### 7.4 Live run 2 (`media/runs/w4_structure_decks_mission_freespace_run2.mp4`)

Same set-up + the gate, with map snapshots every 150 s for offline replay. **Not reached; 443 m in 900 s, closest 4.5 m.** Timeline:
west door -> corridor (0-100 s) -> out by the south decoy door, round the east side, in by the east exit (100-250 s) -> through the
centre rooms to (-5.6, 5.0), the room next to B, **two metres from B's 1.0 m door at (-8, 8)** (~300 s) -> the route over the door
disappeared, and the planner sent the robot out of the building and round the north side to B's (unseen) west wall, where the run
ended. Replay of the snapshots: **B's door opening was filled with confident camera LETHAL cells** (confidence ~0.17, height
+12 cm), the lidar marked only the door jambs. These are almost certainly **flying pixels**: at a depth discontinuity (the door
jamb seen at an angle) a depth camera returns points between the near edge and the far floor; the v2 rules read them as a
10-15 cm step. With the door "closed", freespace correctly looked for another way in - the only unseen candidate was B's outer wall.

### 7.5 Where this leaves exploration under roofs

* The planning strategy behaves as designed live (no target switching; it reached B's neighbour in 300 s, where viewpoints never
  got inside in the earlier live runs except with the island bug).
* The blocker is now PERCEPTION: the UGV camera's ground layer produces false obstacles at edges. Proposed next step (awaiting
  approval): a flying-pixel / veiling-point filter on the UGV depth image before fusion (drop a point whose depth differs from its
  image neighbours by more than a few cm x range, as PCL's `ShadowPoints` filter does), then one live W4-B run.
* Without the ground layer the real UAV scan's false obstacles block the west door (4.3); with it, the camera's own artefacts block
  doors. Either way, the map's false LETHAL cells, not the planner, decide the outcome on W4.
