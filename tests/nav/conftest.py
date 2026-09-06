"""Pytest fixture-discovery file for tests/nav/ - same rationale as
tests/emap/conftest.py (fast, ROS-free unit tests shouldn't need a colcon
build first). Puts BOTH src/nav/ and src/emap/ on sys.path, since `nav`'s
own modules import `emap` directly (e.g. `nav.walkability` imports
`emap.traversability.LETHAL`).
"""
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
for _pkg_dir in ("nav", "emap"):
    _src = _REPO_ROOT / "src" / _pkg_dir
    if str(_src) not in sys.path:
        sys.path.insert(0, str(_src))
