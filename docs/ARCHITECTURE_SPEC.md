# RoboRun Architecture Spec — Tap, Tracking, Storage

Status: **MVP implemented** (v0.11.0). The §4 MVP scope shipped: vendored
transport (`roborun/transport/` — general message types, capability matrix,
tap mode), MCAP recorder with chunk-granular hash chain, O(1) seal,
OpenTimestamps anchoring and three-state verify (`roborun/recorder.py`,
`roborun/anchor.py`), MCAP→Observation extraction into the upgraded indexed
SQLite store (`roborun/observations.py`, `roborun/spatial_memory.py`),
R2 sync + Parquet/DuckDB fleet queries (`roborun/r2sync.py`), signed beacons
(`roborun/beacons.py`), and flight deck record / verify badge / clip export.
The "Later" items in §4 remain open. The original problem statement follows.

This describes
how the app *should* look after we fix the three things that block the wedge:
the transport (ros_tap), the recorder (tracking + tamper evidence), and the
data layer (query at scale + cross robot sharing).

---

## 0. Current state, honestly

Three storage systems exist today and **none of them know about each other**:

| System | File | What it stores | How it's queried | Integrity |
|---|---|---|---|---|
| Event journal | `events.py` → `~/.roborun/runs/<run>/run.jsonl` | agent/MCP/ROS/detection events as hash chained JSONL | in memory deque(500) + SSE | hash chain + seal (`integrity.py`) |
| Spatial memory | `spatial_memory.py` → `.roborun/spatial_memory.db` | JPEG thumbnails, CLIP embeddings, YOLO detections, x/y/z | SQLite, full scan cosine, `LIKE` for YOLO | none |
| Datasets | `dataset.py` → `./datasets/<name>/<ep>/` | full JPEG frames (q95) + `episode.json` | directory walk | none |

The consequences:

- The "black box" (the sealed thing) contains *events*, not the images, detections, or video. So the tamper evident artifact does not actually commit the perception data, which is the evidence that matters.
- Spatial memory holds the images and detections but has **no integrity** at all, and its query path is a full table scan for CLIP (in memory numpy matmul) and a `LIKE '%"label"%'` string match for YOLO. That falls over past ~100k rows and cannot go cross robot.
- Datasets is a third copy of the same frames at q95, disconnected from both.
- The transport, `ros_tap`, is an external dependency (`ros_tap>=0.1`) that is unreliable: DDS direct only hardcodes a `Twist` publish and topic discovery, with no general message deserialization. Everything real ("subscribe_duration", services, actions) falls back to rosbridge, which requires a bridge running on the robot. The "zero config DDS" promise is mostly unmet.

The fix is not three better systems. It is **one recording substrate (MCAP), one transport abstraction (ros_tap, vendored and real), and one data layer that is nothing but local files plus R2.** No brokers, no database servers, no standing services of any kind. The hot query layer is embedded libraries reading files; cross robot is R2 as a shared blackboard. Everything below collapses toward that.

---

## 1. ros_tap — the transport layer

### 1.1 What it must become

`ros_tap` should be a real, vendored, in repo transport abstraction with one
interface and three backends behind it, plus a passive recording mode. It stops
being a flaky external package and becomes the thing that both drives the robot
*and* feeds the recorder.

```
Transport (interface)
  ├─ discover(domain, timeout) -> [Robot{id, host, topics, types, transport}]
  ├─ topics() / type_of(topic) / message_schema(type)
  ├─ subscribe(topic, cb)   / unsubscribe(topic)
  ├─ publish(topic, msg)    / call_service / send_goal (action)
  └─ capabilities() -> {pub, sub, services, actions, params, types}

Backends:
  A. DDS direct  (CycloneDDS)  — zero config discovery + pub/sub
  B. rosbridge   (websocket)   — services/actions/params + any type when a bridge runs
  C. native      (rclpy)       — when running on-robot inside a ROS env
```

### 1.2 The real fixes (not cosmetic)

1. **General message types, not just Twist.** Today DDS can only publish `Twist`. The fix is dynamic type support: discover type definitions via DDS XTypes/TypeObject where the robot advertises them, fall back to a bundled schema set for the common families (`geometry_msgs`, `sensor_msgs`, `vision_msgs`, `nav_msgs`, `std_msgs`, `tf2_msgs`, `sensor_msgs/CompressedImage`, `foxglove.CompressedVideo`). Without this, recording is impossible because we can't deserialize what we subscribe to.
2. **Capability matrix surfaced to the agent.** The LLM should never call a service over a DDS only connection and get a confusing error. `capabilities()` tells the agent and the UI exactly what this robot supports right now, so tools degrade gracefully.
3. **Robust discovery + liveness.** DDS participant discovery across a domain range, deduped against rosbridge, cached, with a heartbeat so a robot dropping off the graph is detected and surfaced (this also feeds fleet status).
4. **Tap mode (the bridge to storage).** A passive subscriber that taps a configured set of topics straight into the MCAP recorder *without the LLM in the loop*. This is the literal meaning of "ros_tap": tap the ROS graph into the black box. It runs at full rate, the agent runs at its own pace, and both land in the same run.
5. **Exposure scanner reuse.** The same discovery primitive powers the "is your robot exposed to the internet" tool from the launch plan. One scanner, two uses (connect for good, audit for safety), zero new attack surface against named companies.

### 1.3 Capability matrix (target)

| Capability | DDS direct | rosbridge | native rclpy |
|---|---|---|---|
| discovery (zero config) | yes | partial (needs host) | yes |
| subscribe / record | yes (common types) | yes | yes |
| publish / move | yes | yes | yes |
| services / params | no | yes | yes |
| actions (goals) | no | yes | yes |
| arbitrary custom types | via XTypes only | yes | yes |

The agent reads this and adapts. No silent failures.

---

## 2. roborun recorder — tracking + tamper evidence over MCAP

### 2.1 One substrate: MCAP

Every run is **one MCAP file**, the single source of truth for that run. It is
schema agnostic and encoding agnostic, so all data types share one container as
separate channels:

| Channel | Schema | Source |
|---|---|---|
| `/camera/<name>` | `sensor_msgs/CompressedImage` (keyframes) **or** `foxglove.CompressedVideo` (H.264/H.265 stream) | ros_tap camera tap / webcam |
| `/detections/<name>` | `vision_msgs/Detection2DArray` (+ `Detection3DArray`) | YOLO |
| `/clip/embeddings` | custom protobuf/JSON `{vec[], frame_ref, label?}` | CLIP encoder |
| `/agent/events` | JSON `{type, source, title, detail, prev}` (today's `run.jsonl` events) | events.py emitters |
| `/tf`, `/odom`, `/pose` | standard `tf2_msgs`, `nav_msgs/Odometry`, `geometry_msgs/PoseStamped` | ros_tap pose tap |

Video is the bulk win: moving cameras from per frame JPEG (`CompressedImage`)
to a real codec channel (`foxglove.CompressedVideo`) is 10x to 50x smaller and
renders natively in Foxglove Studio. Out of band video (phone, GoPro, existing
mp4) stays an external file, hashed in segments, with the segment hashes
committed to the chain.

This deletes the three way split: `events.py`, `spatial_memory.py`, and
`dataset.py` all stop being separate stores. The MCAP is the recording;
everything else is derived from it (see §3).

### 2.2 Integrity over MCAP (fixes the reseal hole)

The hash chain moves **out of a private JSONL and onto the MCAP itself**:

- **Leaf = MCAP chunk.** As the writer flushes each chunk (chunks are MCAP's native, compressed, indexed unit), hash it and link it: `chain[i] = H(chunk_bytes_i, chain[i-1])`. The chain lives in a small sidecar index `(chunk_offset, chunk_hash, prev)`, not injected into messages we don't own.
- **Seal = Merkle root over chunk hashes + anchor proof.** Drop the stored per event hash list from the seal (it makes the seal grow O(n) for nothing). The root verifies; to localize a failure we recompute the tree at verify time. Seal becomes O(1) in size: roots, counts, timestamps, anchor proof. A few KB regardless of run length.
- **Anchor (the part that makes verify mean anything).** On seal, stamp the root with **OpenTimestamps** (trustless, Bitcoin, tiny `.ots`, async upgrade) and optionally an **RFC 3161 TSA token** (instant, trusts a named authority, good for live demos). While the run is live, anchor chain heads opportunistically so a long run is pinned to an external clock before it ends.
- **Honest states from verify.** Not a binary. Three outcomes: `verified + anchored` (unchanged since an outside clock witnessed it), `internally consistent, unanchored` (chain intact but never externally timestamped, e.g. offline robot), and `broken` (which chunk, which channel, which message). Offline runs anchor opportunistically when connectivity returns; the seal records its own anchor status.

What this proves: the recorded run, **including the images, detections, video, and agent decisions**, has not been altered since a moment an external clock witnessed. That is the claim the black box needs and the one it cannot make today.

### 2.3 Run lifecycle

```
open run  → MCAP writer opens runs/<robot>/<run_id>.mcap, manifest links prev run's root
record    → ros_tap taps topics at full rate; agent emits /agent/events; CLIP/YOLO write their channels
           periodic chunk seal; chain heads anchored opportunistically
close     → final chunk flush → Merkle root → OpenTimestamps stamp → .seal + .ots written
index     → MCAP streamed into the hot store (§3); MCAP+seal+ots uploaded to object store
```

Runs already link end to end via `manifest.prev_run` (events.py does this). Keep
it: the fleet's history is a chain of sealed runs.

### 2.4 Flight deck (how it should look)

- Replays straight from MCAP, so the camera and detection overlays are native Foxglove style views, not custom UI we maintain.
- Shows the verify badge live: anchored / unanchored / broken, with the external timestamp when present.
- **Cut a verified clip.** Pick a window, export the segment plus a proof that those exact frames are unaltered. This is the shareable artifact that ties the security wedge to the viral demo: a tamper evident clip you can post, not a CLI nobody watches.

---

## 3. Storage — query at scale, store a ton, cross robot

**Hard constraint: local files and R2 only. No other infra.** No brokers, no
database servers, no vector DB services, nothing to operate. The hot query layer
is an *embedded* engine reading files in process. Cross robot is R2 used as a
shared blackboard, not a message bus.

Two tiers and a sync, not three services. Cold holds everything cheaply on R2;
hot is a queryable index that is just files; "cross robot" is both robots
reading and writing the same R2 prefix.

### 3.1 Tier 1 — cold: object store is the source of truth

MCAP runs live in R2/S3 (you already run R2 heavily). Append only, cheap,
effectively unbounded. This is "store a ton of data."

```
runs/<robot_id>/<run_id>.mcap      # the recording (heavy)
runs/<robot_id>/<run_id>.seal      # Merkle root + counts (KB)
runs/<robot_id>/<run_id>.ots       # OpenTimestamps proof (KB)
index/<robot_id>/<date>.parquet    # derived columnar rows (see 3.2)
```

The MCAP is never queried directly for analytics. It is the cold archive you
replay or re verify; queries hit the derived index.

### 3.2 Tier 2 — hot: a columnar + vector index derived from MCAP

The atomic unit is one **Observation**, the join key across all tiers:

```
Observation {
  obs_id, robot_id, run_id, ts,
  pose { x, y, z, frame_id },
  detections [ { label, score, bbox } ],
  clip_embedding [float32; 512],
  frame_ref { mcap_offset, thumb_key? },   # where to pull the full frame
  source
}
```

On run close (and optionally streaming), the MCAP is extracted into Observation
rows. The hot store stays **SQLite**, which is already a dependency (stdlib) and
already most of what `spatial_memory.py` does. The fixes are the weak spots, all
without adding a service:

- **YOLO: stop using `LIKE`.** Today `search_yolo` does `detections LIKE '%"label"%'`, a full table string scan. Normalize: a `detections(obs_id, label, score, bbox)` child table with an index on `label`. Now label queries are indexed lookups across millions of rows.
- **CLIP: keep numpy for now, add an embedded ANN only if scale demands it.** The current in memory cosine matmul is genuinely fine to ~1M vectors. If it outgrows that, add **sqlite-vec** (an embedded SQLite *extension*: vectors live in the same `.db` file, ANN in process, no server) or **hnswlib** (a single embedded library, an index file on disk). Both honor the constraint: a library and a file, nothing running.
- **Spatial / time: already SQL, just index it.** `search_nearby` and time range are already SQL filters; add the composite indexes and they hold up.

For ad hoc analytics across the whole fleet ("count distinct zones where any
robot saw a forklift last week"), export Observation rows to **Parquet** in R2
and query them with **DuckDB**, which is also embedded (a library, no daemon)
and reads Parquet directly from R2. This is optional and on demand; it is not a
running service.

Why this shape: it answers "query it" (indexed SQL + optional embedded ANN +
optional DuckDB over Parquet), "store a ton" (MCAP and Parquet in R2), and "no
infra" (every engine here is a library reading files; the only durable store is
R2). The hot index is derived and disposable: rebuild it from the cold MCAP any
time.

### 3.3 Cross robot — R2 as a shared blackboard (no bus)

No broker. Robots communicate by reading and writing the **same R2 prefixes**.
This is eventually consistent shared memory, which is the right model for a
fleet and needs zero infra.

**(a) Shared memory (asynchronous).** Every robot's Parquet index exports land
in one shared R2 dataset, partitioned by `robot_id` and date:
`index/<robot_id>/<date>.parquet`. Any robot or the cloud agent queries the
union with DuckDB pointed at `index/*/*.parquet` in R2. `search_clip("red mug")`
returns hits from the whole fleet; `search_nearby` spans robots. This already
half exists (there is a `robot_id` column); the change is that the durable index
lives in R2, not in one robot's local `.db`. Robot B asks "has anyone seen X
near here" by reading the shared prefix.

**(b) Live-ish beacons (polling, still no broker).** For "I see X at (x,y) now"
and task handoffs, robots write small signed beacon files to a shared prefix,
`beacons/<robot_id>/<ts>.json` (label + pose + embedding hash + MCAP ref, **no
images**). Other robots poll that prefix on an interval, or react to R2 event
notifications if enabled. It is not sub second, but it is shared awareness with
nothing to run. A robot that cares about a beacon pulls the full frame from the
MCAP in R2 on demand, so the beacons stay tiny and the heavy data stays lazy.
Old beacons are swept by lifecycle policy on the bucket.

**(c) Trust, where the wedge and the fleet meet.** Each robot signs its beacons
and its sealed runs with the same Ed25519 identity from `integrity.py`. A cross
robot claim ("I saw a person in the aisle") is itself verifiable and anchored:
robot B trusts robot A's observation because it is signed by A and its source
run is sealed and timestamped. The black box stops being a per robot compliance
feature and becomes the **trust layer for fleet coordination** — the reason a
multi robot system can act on another robot's word. Bigger story than "logs for
the EU AI Act," same primitives, and it needs no infra beyond R2 holding signed
files.

**The honest tradeoff:** R2 polling gives you seconds of latency, not
milliseconds. That is fine for shared spatial memory and task handoff, and wrong
for tight real time control loops (one robot reacting to another's motion in
under 100ms). If that ever becomes a real need, it is the one and only place a
broker would earn its keep, and it would be an explicit, deliberate infra
decision made later, not a default. The MVP does not need it.

### 3.4 What gets deleted or merged

- `dataset.py` (episode JPEG recorder): folded into MCAP recording. Episodes become labeled runs or labeled segments of runs.
- `spatial_memory.py`: kept and upgraded, not replaced. Same SQLite engine, fixed schema (indexed detections, optional embedded ANN), and its durable copy syncs to R2 alongside the MCAP. It stops being the *only* copy, not the SQLite.
- `events.py` private `run.jsonl`: becomes the `/agent/events` channel *inside* the MCAP, so agent decisions are committed alongside perception. The hash chain logic moves to chunk granularity in the sidecar.

Three systems become one substrate (MCAP) plus one derived index (SQLite,
optionally exported to Parquet), all of it living in local files and R2. Less
code, not more, and nothing to operate.

---

## 4. Phasing (initial version vs later)

**MVP (ship behind the launch):**

- ros_tap: vendor a minimal but real DDS + rosbridge transport with the common message families and tap to recorder mode. Kill the Twist only limitation.
- recorder: MCAP writer with the five channels, chunk hash chain in a sidecar, O(1) seal, OpenTimestamps anchor, three state verify.
- storage: MCAP → Observation extractor on close into the upgraded SQLite (indexed detections, numpy CLIP); R2 upload of MCAP + seal + ots + the `.db`; CLIP/YOLO/nearby/time queries served locally.
- cross robot: shared R2 prefixes (`index/*/*.parquet` queried by DuckDB on demand, `beacons/` polled). Signed beacons. No broker.
- flight deck: replay from MCAP, verify badge, verified clip export.

**Later:**

- Embedded ANN (sqlite-vec or hnswlib) only if vector count outgrows numpy.
- Streaming extraction (index while recording, not only on close).
- Native rclpy backend for on robot deployment.
- Foxglove deep integration, TSA + OTS dual anchoring in prod.
- A broker for sub second cross robot control loops — only if that need ever becomes real, as an explicit infra decision, never a default.

---

## 5. Open decisions (need a call before building)

1. **Anchor:** OpenTimestamps only (purest trustless story, hours to confirm) vs OTS + RFC 3161 TSA (instant live demo proof, trusts a named authority). Both are just files, so both honor the constraint. Leaning both.
2. **Vector search at scale:** stay on numpy until it hurts (simplest), or add sqlite-vec (extension, vectors in the same `.db`) vs hnswlib (library + index file) when it does. All three are embedded, no service. Leaning numpy for MVP, sqlite-vec when needed.
3. **Fleet analytics:** DuckDB over Parquet in R2 (embedded, on demand) is the recommended path since it adds no service. Confirm DuckDB is an acceptable library dep, or we keep it to per robot SQLite and merge results in app code.
4. **Video in container vs referenced:** `foxglove.CompressedVideo` channel inside MCAP (one sealed object) vs external mp4 with segment hashes. Support both; default to in container for robot cameras.
5. **Keyframe cadence + segment granularity:** tighter keyframes cost size, buy excerptability for verified clips. Pick a default (e.g. 2s GOP).
