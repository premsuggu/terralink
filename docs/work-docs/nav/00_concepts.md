# nav: from-scratch concepts

Read once, alongside `emap`'s own `docs/work-docs/emap/00_concepts.md` (this package builds directly on `emap`'s elevation/traversability layers and doesn't re-explain them here).

## 1. Why not "flat elevation = walkable"?

The obvious-looking rule is wrong for a reason `emap` already proved concretely: its construction-site world's flat ground sits at absolute world z=+1.5, not 0 (a from-scratch heightmap generation choice, see `docs/work-docs/emap/step12_construction_site_world.md`). Any rule based on an *absolute* height number needs re-tuning for every world, and would misclassify an entire flat-but-elevated world as one giant obstacle.

`emap.traversability.compute_traversability` (already built, step 7) sidesteps this by scoring each cell from how it compares to its *neighbors* - slope (rise/run between adjacent cells), step-height (the tallest-vs-shortest cell within a 3x3 neighborhood), and roughness (measurement variance). None of those three numbers care what the absolute height value is - only how much it *changes* nearby. That's what makes it safe to reuse directly as `nav`'s walkability classifier (`nav.walkability.compute_walkable_mask`), with no new "is this flat" logic needed at all.

## 2. Why also check `is_valid`?

A cell `compute_traversability` has genuinely scored `LETHAL` and a cell it's never scored at all can both, mechanically, hold non-LETHAL numbers in memory (the latter defaults to `EASY`, a documented fail-safe for unrelated callers - see `traversability.py`'s own docstring). Treating "never observed" the same as "confirmed safe" would let a planner route a UGV across ground the UAV has literally never looked at - the standard "unknown space is not free space" rule any real occupancy-grid-based planner (Nav2's own costmaps included) already follows. `compute_walkable_mask` enforces this explicitly rather than trusting the default.

## 3. What a PRM (Probabilistic Roadmap) actually does

A PRM builds a graph ("roadmap") over free space, then searches that graph for a path - two separate steps:

1. **Build the roadmap**: scatter random points ("nodes") across the map, keep only the ones landing in free space, then try to connect nearby pairs of nodes with a straight-line "edge" - but only if that straight line doesn't cross any obstacle cell along the way (checked by walking the line one grid cell at a time - `nav.prm_planner._line_is_walkable`). The result is a graph: nodes are places the robot could be, edges are straight-line paths it could safely drive between two of them.
2. **Search the roadmap**: once the query's actual start/goal are added as two more nodes, a shortest-path algorithm (Dijkstra, via `scipy.sparse.csgraph.dijkstra`) finds the lowest-total-distance chain of edges connecting them. The chain of nodes along that path, in order, is the returned route.

Why random sampling instead of, say, a fine regular grid of candidate points: a PRM's whole appeal is that it doesn't need to examine every cell in the map to find a decent route - a modest number of random samples (a few hundred, here) is usually enough to find *a* connecting path through open space, at a fraction of the cost of exhaustively searching the full grid. The tradeoff is that a PRM can occasionally fail to find a path that genuinely exists (bad luck in the random sampling) even when one does - `nav.prm_planner.plan` reports that honestly as `valid=False` rather than pretending certainty a sampling-based method can't actually offer.

## 4. Why d3's own camera-projection math is gone

`d3`'s planner (`processImage.cpp`'s `coordToPixel`/`pixelToCoord`) had to invent a way to convert between world coordinates and image pixels, because its only input was a raw camera image with no metadata about scale or position - it assumed a fixed-altitude, fixed-field-of-view downward camera and did the trigonometry by hand. `emap`'s published map already carries an exact `resolution` (meters/cell) and world-frame center in every `GridMap` message, so `nav` just reuses the exact grid math `emap` itself is built on (`emap.utils.coord_transform.world_to_grid`/`grid_to_world`) - no camera, no altitude assumption, no trigonometry to get right or wrong.
