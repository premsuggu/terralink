# Terrain measurements on the vehicle (C-07, 2026-11-11)

`c07_rubble_lanes.png` - eight rubble lanes in Gazebo (`tests/nav/integration/make_rough_world.py`, driver `ugv_rough.py`), the Jackal drives
each at 0.5 m/s. Bumps up to 8 cm are crossed at normal speed (19-21 s for 9.3 m); from 10 cm the wheels stop, which is the 9 cm step
limit measured earlier. `c07_rubble_lanes_raw.txt` is the driver's own output (map roughness, crossing time, tilt, verdict per lane).
Decision: `max_roughness` stays 0.06 (0.04 would flag the ~3 cm noise of a real UAV map). Full write-up: `docs/work-docs/nav/step14_finalisation.md` section 6.
