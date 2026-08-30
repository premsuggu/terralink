"""Pytest fixture-discovery file for tests/emap/.

The `emap` package normally becomes importable only after `colcon build` +
`source install/local_setup.bash` (that's how a ROS 2 ament_python package
gets installed). But per AGENTS.md, these unit tests are meant to be a FAST,
ROS-free way to check the algorithm code alone - requiring a full colcon
build just to run a NumPy test would defeat that purpose. So instead, we put
`src/emap/` (the folder that directly contains the `emap` Python package)
onto `sys.path` ourselves, here, before any test file runs.
"""
import sys
from pathlib import Path

# This file lives at <repo_root>/tests/emap/conftest.py, so going up two
# directories gets back to <repo_root>, and from there down into
# src/emap/ is where the actual `emap` package folder lives.
_REPO_ROOT = Path(__file__).resolve().parents[2]
_EMAP_SRC = _REPO_ROOT / "src" / "emap"
if str(_EMAP_SRC) not in sys.path:
    sys.path.insert(0, str(_EMAP_SRC))
