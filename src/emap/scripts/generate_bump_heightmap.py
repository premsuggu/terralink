#!/usr/bin/env python3
"""Generate a grayscale PNG heightmap of a single Gaussian bump, for use with
SDF's native `<heightmap>` geometry (see worlds/bump_test.world).

Why a heightmap image instead of a mesh: SDF's `<heightmap>` element takes a
plain grayscale image + a `<size>` (world units) and Ignition builds the
actual 3D terrain from it directly - no mesh-generation library (e.g.
trimesh) needed, and no network/Fuel dependency (see step01's docs for why
we avoid depending on Fuel in this sandbox). Pixel value 0 (black) = the
lowest point of the heightmap's declared height range; 255 (white) = the
highest. We only ever use part of that range (a bump rising from a flat
base), never the extremes, so there's no risk of clipping at pure black/white.

Run once, offline (not at simulation launch time) - this script's PNG output
is committed as a normal repo asset, exactly like the vendored meshes in
models/iris_quad/meshes/ from step 1: `python3 generate_bump_heightmap.py`
"""
from __future__ import annotations

import os

import numpy as np
from PIL import Image

# Physical size this heightmap image will be stretched over when referenced
# with <size>FIELD_SIZE_M FIELD_SIZE_M BUMP_HEIGHT_M</size> in the world file
# - must match exactly, since this script and the world file don't share a
# single source of truth for these numbers (a real project would generate
# the world file's <size> tag from this script's own constants instead of
# duplicating them by hand; kept simple here since this is a one-off test
# terrain, not something we expect to regenerate with different parameters
# often).
FIELD_SIZE_M = 20.0   # matches this project's default map `length` (config/elevation_mapping.yaml)
BUMP_HEIGHT_M = 1.5   # peak height of the bump
BUMP_SIGMA_M = 1.5    # Gaussian "spread" - chosen so the bump's steepest
# slope (which occurs at one sigma from the peak, standard Gaussian
# property) comfortably exceeds this project's own max_slope threshold
# (0.35) - peak_height / (sigma * sqrt(e)) ~= 1.5 / (1.5 * 1.65) ~= 0.61 -
# so flying over this bump is guaranteed to actually exercise the LETHAL
# slope classification, not just the DIFFICULT or EASY ones.

# Ignition/Ogre2 terrain paging expects a heightmap image sized 2^n + 1 on
# each side (a well-known convention carried over from Gazebo Classic) -
# 129 = 2^7 + 1 keeps this comfortably detailed (~16cm/pixel over the 20m
# field) without being unnecessarily large.
IMAGE_SIZE_PX = 129

OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "heightmaps", "gaussian_bump.png"
)


def main() -> None:
    # Pixel coordinates centered on the image, converted to real-world
    # meters using the field size - so `xs`/`ys` range from -10m to +10m
    # for FIELD_SIZE_M=20, matching how the heightmap will actually be
    # positioned (centered at the world origin) in bump_test.world.
    coords = np.linspace(-FIELD_SIZE_M / 2, FIELD_SIZE_M / 2, IMAGE_SIZE_PX)
    xs, ys = np.meshgrid(coords, coords)

    # Standard 2D Gaussian bump, peak 1.0 at the center, decaying to ~0 at
    # the edges of the field (at 20m/2=10m from center, and sigma=1.5m,
    # we're ~6.7 sigma out - the Gaussian is astronomically close to zero
    # there, so the heightmap's edges are flat, avoiding any visible seam
    # where this terrain patch would meet the surrounding flat ground).
    height_fraction = np.exp(-(xs**2 + ys**2) / (2 * BUMP_SIGMA_M**2))

    # Scale to 0-255: since height_fraction's minimum (at the corners) is
    # already ~0, black (0) represents "flat ground" and white (255) is the
    # bump's peak - exactly the 0..BUMP_HEIGHT_M range <heightmap><size>'s
    # z-component will stretch this image over in the world file.
    pixels = np.clip(np.round(height_fraction * 255), 0, 255).astype(np.uint8)

    # flipud: image row 0 is conventionally the TOP of the picture (matching
    # how image viewers/PNG display it), but we built `ys` increasing
    # downward through the array in the same direction `np.meshgrid` laid
    # out rows - flipping vertically here keeps the heightmap's north/south
    # orientation matching the world's Y axis instead of being mirrored.
    image = Image.fromarray(np.flipud(pixels), mode="L")

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    image.save(OUTPUT_PATH)
    print(f"Wrote {IMAGE_SIZE_PX}x{IMAGE_SIZE_PX} heightmap to {OUTPUT_PATH}")
    print(f"Bump: peak={BUMP_HEIGHT_M}m, sigma={BUMP_SIGMA_M}m, field={FIELD_SIZE_M}m")


if __name__ == "__main__":
    main()
