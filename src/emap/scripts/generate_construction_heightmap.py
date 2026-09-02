#!/usr/bin/env python3
"""Generate a grayscale PNG heightmap for the construction-site world: mostly
flat ground with one dug-out excavation pit (below ground) and one spoil
mound (the dirt piled up from digging it, above ground) - see
worlds/construction_site.world.

This is the same technique as generate_bump_heightmap.py (SDF's native
`<heightmap>` geometry, no mesh library needed - see that script's docstring
for the full rationale), extended in one way: that script only ever produces
heights ABOVE a flat base (black=flat, white=peak). A construction site needs
height to go BOTH ways from flat ground - this script picks a MIDDLE gray
value to represent "flat", with the pit dipping toward black and the mound
rising toward white.

A REAL BUG found live is why this does NOT try to make "flat" line up with
world z=0 via a <pose> offset on the heightmap link, even though that was
the first, more obviously "tidy" design: Ignition's <heightmap> geometry
was found (via a live Gazebo debug log reporting the physics engine's own
computed collision bounds, and confirmed by the depth camera measuring the
pit's true depth as ~0 instead of the intended -1.5) to IGNORE a non-zero
link-level <pose> Z offset entirely for a heightmap's collision (and,
consistently, its rendered) geometry - the terrain always occupies exactly
[0, size.z] in world Z, no matter what pose is set on it.

So instead: the heightmap's raw [0, TOTAL_HEIGHT_RANGE_M] range is used
as-is, unshifted (pose left at the SDF default identity, exactly like
bump_test.world's own working bump_terrain), and "flat ground" simply lives
at world z=+PIT_DEPTH_M instead of 0 - construction_site.world places every
other model (crane, excavator, people, building, the UAV's own spawn point)
at that same +PIT_DEPTH_M baseline instead of 0. Every RELATIVE height (the
pit's depth below flat, the mound's height above flat) is exactly as
intended either way - only the absolute datum moved, which costs nothing
for this project's purposes (nothing depends on "flat ground = world z 0"
as an invariant).
"""
from __future__ import annotations

import os

import numpy as np
from PIL import Image

FIELD_SIZE_M = 30.0     # bigger than bump_test.world's 20m - room for the
# crane, excavator, building frame, pit, and mound without crowding.
PIT_DEPTH_M = 1.5       # how far below flat ground the excavation goes.
MOUND_HEIGHT_M = 2.0    # how tall the spoil pile (dug-out dirt) rises.
TOTAL_HEIGHT_RANGE_M = PIT_DEPTH_M + MOUND_HEIGHT_M  # the <size> z-component
# construction_site.world's <heightmap> must use.

# Where "flat ground" sits within the [0, TOTAL_HEIGHT_RANGE_M] range this
# heightmap covers - NOT the midpoint unless PIT_DEPTH_M and MOUND_HEIGHT_M
# happen to be equal. Since the terrain is placed unshifted (see the
# docstring above), this fraction times TOTAL_HEIGHT_RANGE_M is also
# "flat ground"'s actual, absolute world Z - i.e. exactly PIT_DEPTH_M.
FLAT_LEVEL_FRACTION = PIT_DEPTH_M / TOTAL_HEIGHT_RANGE_M

PIT_CENTER = (-4.0, 2.0)   # meters from the field center
PIT_SIZE = (6.0, 4.0)      # (x, y) footprint of the excavation, meters
PIT_EDGE_SOFTEN_M = 0.4    # how many meters the pit's walls slope over,
# rather than dropping as an instant vertical cliff - a real dug pit has
# sloped/stepped walls, not a knife-edge, and a perfectly vertical drop would
# also be numerically ambiguous for world_to_grid's nearest-cell rounding.

MOUND_CENTER = (5.0, -4.0)  # meters from the field center - off to the side,
# as if the excavated dirt was piled up out of the way of the dig.
MOUND_SIGMA_M = 2.0         # same Gaussian-bump shape as generate_bump_heightmap.py

IMAGE_SIZE_PX = 257  # 2^8 + 1 - same "2^n + 1" convention as the bump
# heightmap (see that script's docstring), sized up slightly for this
# larger 30m field to keep a similar per-pixel resolution (~12cm/pixel).

OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "heightmaps", "construction_terrain.png"
)


def smooth_rect_depression(xs, ys, center, size, soften) -> np.ndarray:
    """1.0 at the pit's center, smoothly falling to 0.0 outside its footprint
    (plus a `soften`-meter-wide sloped edge) - a smoothed rectangle rather
    than a hard-edged one, using the same "distance past the boundary, run
    through a smooth 0..1 falloff" idea as a rounded-rectangle SDF (signed
    distance function), just written directly as elementwise NumPy instead
    of a generic distance-field library.
    """
    dx = np.maximum(np.abs(xs - center[0]) - size[0] / 2, 0.0)
    dy = np.maximum(np.abs(ys - center[1]) - size[1] / 2, 0.0)
    dist_outside = np.sqrt(dx**2 + dy**2)  # 0 inside the rectangle, growing outside it
    # Smoothstep-style falloff over `soften` meters - 1.0 right at/inside the
    # rectangle, 0.0 once `soften` meters past its edge, smooth in between
    # (no sharp derivative discontinuity, unlike a plain linear ramp).
    t = np.clip(1.0 - dist_outside / soften, 0.0, 1.0)
    return t * t * (3 - 2 * t)


def main() -> None:
    coords = np.linspace(-FIELD_SIZE_M / 2, FIELD_SIZE_M / 2, IMAGE_SIZE_PX)
    xs, ys = np.meshgrid(coords, coords)

    pit_depression = smooth_rect_depression(xs, ys, PIT_CENTER, PIT_SIZE, PIT_EDGE_SOFTEN_M)
    mound_bump = np.exp(
        -((xs - MOUND_CENTER[0]) ** 2 + (ys - MOUND_CENTER[1]) ** 2) / (2 * MOUND_SIGMA_M**2)
    )

    # Start everywhere at FLAT_LEVEL_FRACTION (flat ground), subtract the pit
    # (only ever pulls height DOWN, toward 0), add the mound (only ever pushes
    # height UP, toward 1) - the two features don't overlap in this layout
    # (PIT_CENTER and MOUND_CENTER are far enough apart, verified below) so
    # there's no need to worry about them fighting over the same pixels.
    height_fraction = FLAT_LEVEL_FRACTION - pit_depression * FLAT_LEVEL_FRACTION \
        + mound_bump * (1.0 - FLAT_LEVEL_FRACTION)

    pixels = np.clip(np.round(height_fraction * 255), 0, 255).astype(np.uint8)
    image = Image.fromarray(np.flipud(pixels), mode="L")  # flipud: see
    # generate_bump_heightmap.py's identical line for why.

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    image.save(OUTPUT_PATH)
    print(f"Wrote {IMAGE_SIZE_PX}x{IMAGE_SIZE_PX} heightmap to {OUTPUT_PATH}")
    print(f"Field: {FIELD_SIZE_M}m, pit depth={PIT_DEPTH_M}m @ {PIT_CENTER}, "
          f"mound height={MOUND_HEIGHT_M}m @ {MOUND_CENTER}")
    print(f"heightmap <size> z must be {TOTAL_HEIGHT_RANGE_M}, model pose stays at the SDF default "
          f"(no offset - see docstring); flat ground sits at absolute world z={PIT_DEPTH_M}, "
          f"pit bottom at z=0, mound peak at z={TOTAL_HEIGHT_RANGE_M}")


if __name__ == "__main__":
    main()
