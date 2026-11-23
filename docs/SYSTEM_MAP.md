# RoboRun system map

*How the pieces fit, for the three ways people use it. One loop, three modes.*

## The one loop
```
 source ─▶ YOLO + CLIP ─▶ sealed MCAP (the black box) ─▶ live index ─▶ search/analyze
 (sim/robot/cam)            recorder.py        spatial_memory.py     session.search
```
Everything is the same loop whether the source is a sim, a robot, or a webcam.
`PerceptionSession.for_mode("sim"|"robot"|"production")` runs it (`session.py`).

## Three usage modes (what runs where)
| You are… | You use… | Entry points |
|---|---|---|
| **a front-end user** | the browser dashboards | `/` cockpit · `/scenarios` · `/search` · `/timeline` · `/analytics` · `/fleet` · `/run` (linked from the cockpit ▤ VIEWS menu) |
| **a local-runner operator** | the `roborun` CLI + Python handle | `roborun` (server) · `roborun connect <ip>` · `roborun search <q>` · `roborun scenarios [run\|suite]` · `behaviors/*.py` with the `robot.*` handle |
| **a fleet (many robots)** | shared R2 + signed beacons | each robot runs the loop; `index/*/*.parquet` + `beacons/` shared via R2; `/analytics` fleet panel + cross-robot `/search` |

## Where things live (the map)
- **Record everything** → `recorder.py` (MCAP channels: camera/detections/clip/pose/
  `/cmd`/telemetry/gps/cloud), `retention.py` (GC), `anchor.py` (RFC 3161 seal).
- **Perceive** → `webcam.py`/`ros_camera.py`/`synthetic_camera.py`, `cameras.py`
  (multi-cam), `models.py` (YOLO+CLIP), `depth.py`/`scene_builder.py`.
- **Index + search over time** → `spatial_memory.py` (SQLite + numpy/sqlite-vec ANN),
  `session.search`, `observations.py` (extract + fleet DuckDB).
- **Sim** → `simulator.py` (MuJoCo), `sim_backend.py` (sense via handle),
  `mjx_env.py` (vectorized), `gz.py`/`gz_mujoco.py` (Gazebo).
- **Drive** → `behaviors.py` (`robot.*` handle, hot-reload), `transport/` + `rosbridge.py`
  (ROS 1 & ROS 2), `connect.py`.
- **Evaluate** → `scenario.py` (scored runs + suites), `scenario_defs.py` (runnable +
  `run_matrix`/`regression_gate`), `demo_scenarios.py`.
- **Agent** → `agent.py` (deck command bar), `ros_mcp.py` (MCP tools incl.
  `recall_place`), `skills/` (GitHub-installable).
- **View / analyze** → `routes/scenarios.py`, `routes/search.py`, `run_series.py`
  (`/run` synced playback), `web/*.html`.

## The contract (why one file works everywhere)
A behavior written against `robot.see/move/lidar/pose/goto/go_to_place` runs
unchanged on the arena, MuJoCo, MJX, a gz world, and a real ROS 1/2 robot — the
primitives are schema-identical per backend (`docs/SIM_SPEC.md`,
`tests/test_contract.py`). Scale-up (MJX) and fidelity (gz/real) ride on top.

## Keep it lean
Optional heavy deps are extras (`[vision] [sim] [ros] [fleet] [ann] [video] [mjx]`);
the core is 3 deps. The index is derived/rebuildable from the MCAP. If a feature
needs a new subsystem, first check it can't compose existing primitives (semantic
nav = recall + goto; the data layer is one Observation row).
```
