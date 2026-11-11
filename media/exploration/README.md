# Exploration path pictures (offline simulator)

How each exploration strategy drives to one goal. One panel per strategy:
- true walls in black;
- the driven path coloured from the start (dark blue) to the end (yellow);
- green square = start, red star = goal.

The panel title says whether the goal was reached and how many metres were driven. Explained in
`docs/work-docs/nav/step13_exploration_wallfollow.md`.

| picture | world, goal, map | what to look at |
|---|---|---|
| `w4_noUAV_goalD.png` | W4, goal D (outside, south of the building), no UAV map | freespace drives into the building and turns back (55 m); the exit list does not (29 m); Bug1 circles rooms until 600 m |
| `w4_noUAV_goalB.png` | W4, goal B (inside a room), no UAV map | freespace and the exit list take nearly the shortest route (98-99 m); Bug1/Bug2 go round every room |
| `w4_realUAV_goalB.png` | W4, goal B, real UAV map + camera layer | with a map, today's planner is best (98 m); the exit list commits to a worse exit (158 m) |
| `w2_noUAV_goalC.png` | W2 container yard, goal C, no UAV map | today's planner gets stuck next to the goal; freespace drives straight there (47 m) |

Panels:
- **today's planner** = the live configuration (A* + viewpoints + freespace under roofs);
- **freespace everywhere** = plan as if unseen cells were free;
- **exit list** = best first or newest first (DFS);
- **Bug1 / Bug2** = wall following.

Simulator limits (doors about 0.4 m narrower than in Gazebo, a perfect camera, no false obstacles) are listed in step13
section 3.3.

Regenerate (from `src/nav/scripts/`, after sourcing ROS and `install/local_setup.bash`):

```bash
python3 render_exploration_paths.py --world w4_structure_decks --map none --goal D --out ../../../media/exploration/w4_noUAV_goalD.png
python3 render_exploration_paths.py --world w4_structure_decks --map none --goal B --out ../../../media/exploration/w4_noUAV_goalB.png
python3 render_exploration_paths.py --world w4_structure_decks --map uavscan --goal B --out ../../../media/exploration/w4_realUAV_goalB.png
python3 render_exploration_paths.py --world w2_container_yard --map none --goal C --out ../../../media/exploration/w2_noUAV_goalC.png
```

## EX-07: no UAV map = freespace everywhere (2026-11-11)

`ex07_no_uav_freespace.png` / `.md` (raw log `ex07_no_uav_freespace_raw.log`): all five worlds, every goal, NO UAV map, offline simulator.
"before" = freespace only under roofs (step12), "after" = also in the open when the UAV saw < 2 % of the grid (`nav/freespace.py`).

| | before | after |
|---|---|---|
| goals reached (30 goals) | 16 | **19** |
| target switches (dithering) | 788 | **0** |

Notes: W5 goals 4-8 and a few W1/W2 goals are not reachable in the simulator (doors about 0.4 m narrower than Gazebo, see step13 3.3), so
"reached" never gets to 30. With a real UAV map nothing changes (the switch only fires below 2 % UAV coverage). Rebuild with
`python3 src/nav/scripts/summarize_eval.py <raw.log> <out-prefix> --worlds ... --a adopted_underroof --b adopted`.
