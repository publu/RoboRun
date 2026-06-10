# Speed layers — the reflex architecture

RoboRun's organizing principle is **separation by latency**, like a nervous
system: the hand pulls away from the fire before the brain knows there is a
fire. Every component belongs to exactly one layer, and the two contracts at
the bottom of this page are what keep the layers from corrupting each other.

| Layer | Budget | What runs here | Today |
|---|---|---|---|
| **L0 reflex** | inline, µs | safety clamps, estop, (later: bumper/cliff stop) | `Robot.move()` clamps every command before any transport; `estop` MCP tool |
| **L1 spinal** | 10–50 Hz | behaviors: pure Python over *already-computed* perception — boxes, distances, state machines | `@behavior(hz=10)`, `robot.see()` reading cached detections |
| **L2 perception** | own cadence | model inference on CPU/GPU: YOLO, CLIP, depth, JEPA encoders — each publishes latest-value snapshots | webcam pipeline → detections cache, `/tmp/roborun_state.json`, recorder channels |
| **L3 local brain** | ~1 s | small local LLM: narration, supervisory checks, anomaly flags | `robot.ask()` → Ollama; `@behavior(every=10)` loops |
| **L4 remote brain** | seconds–minutes | big LLM / MCP agents: write and edit behavior files, set targets, enable/disable skills, diagnose | MCP server, hot-reload `behaviors/`, skills |
| **L5 fleet** | minutes, async | cross-robot memory and signals, eventually consistent | Ed25519 beacons + R2 blackboard, Parquet/DuckDB fleet queries |

## Contract 1: lower layers never wait on higher layers

- L1 loops read **snapshots** (latest value wins), never call inference, and
  never block on the network. `robot.see()` is a cache read by design — keep
  it that way.
- `robot.ask()` is forbidden in `hz=` loops; it belongs in `every=` loops or
  tools. (Today this is convention; the runner can warn on tick overrun
  later.)
- L2 models run at whatever rate the hardware allows and publish; they do
  not push into behavior loops.
- Nothing anywhere blocks on L5. A robot with no connectivity is a fully
  functioning robot whose fleet layer is paused (same rule the recorder
  already follows: seal offline, anchor when connectivity returns).

## Contract 2: higher layers act by *parameterizing* lower layers, never by reaching down

The LLM does not drive motors. It writes a behavior file, sets a target
(`robot.remember("patrol_target", ...)`), or toggles a behavior — and the
10 Hz loop does the driving through the L0 clamps. This is what makes LLM
latency and LLM failure harmless: if L4 dies mid-thought, L1 keeps running
and L0 keeps clamping.

Downward = config, targets, code. Upward = snapshots, events, recordings.
Both directions are *data*, so every layer interaction lands in the MCAP
recorder and is replayable.

## Where memory fits

- `robot.remember/recall` is **L1 scratch** — a tiny persistent KV so a
  behavior survives restarts (patrol index, last-seen position, calibration
  offset). The spinal cord's notepad, not "memory" in the cognitive sense.
- `robot.state` is per-process L1 state (gone on restart).
- The **observations index** (SQLite/Parquet derived from sealed MCAP runs)
  is the real long-term memory: queryable history at L3/L4 ("have I seen
  this before"), fleet-wide at L5.

## Fleet (L5): architected now, built later

The rule that avoids a future refactor: **all cross-robot communication goes
through one interface — signed, timestamped, idempotent messages on a shared
blackboard** (today: beacons in R2; `beacons.py` already drops forged
signatures on poll). No robot ever opens a socket to another robot.

Because the interface is "write a signed blob / poll for signed blobs",
swapping the transport later (direct DDS between robots on a LAN, a relay,
anything) is a new backend behind the same interface — the same trick the
transport layer already plays with DDS/rosbridge/rclpy. Fleet consumers to
build when needed, with no architectural change: shared spatial memory
("robot B has seen the red mug"), task handoff, fleet-wide alerts.

## Litmus tests for new code

1. Could this block a `hz=` loop? Then it belongs in L2+ publishing a
   snapshot, or in an `every=` loop.
2. Does an LLM output touch an actuator without passing through a behavior
   and the L0 clamps? Rejected.
3. Does anything fail when the network is gone? Then it's mis-layered —
   only L4/L5 may degrade, and only to "paused".
4. Does a robot talk to another robot except through the signed blackboard?
   Rejected.
