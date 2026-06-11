# RoboRun Sim — real physics, one behavior contract

Status: implementing. Supersedes the hand-rolled arena internals; the
product framing in ARENA_SPEC.md (five users, ladders, scoring, MCP flow)
still stands. This spec covers the two things the arena got wrong:
interactions were faked, and portability was a promise instead of a
contract.

## The contract (the only thing that matters)

A policy file written against the sim runs **unchanged** on any robot on
the supported list. Not "similar API" — the same file. This holds because
the handle is split into two layers:

**Primitives** — backend-specific, schema-identical. Every backend
(arena, MuJoCo, rosbridge, DDS, native rclpy) must provide exactly these,
in these units and frames:

| Primitive | Schema | Real-robot source | Arena source |
|---|---|---|---|
| `pose()` | `{x, z, heading}` m/rad, CCW yaw, +x forward at heading 0; aerial adds `y` | odom (adapter maps ROS x,y → handle x,z) | physics body transform |
| `lidar()` | 36 floats, meters, `[0]` dead ahead, CCW | `/scan` resampled to 36 sectors | 36 raycasts through the physics world |
| `see(label)` | `[{label, confidence, bbox[4], distance}]` | YOLO+CLIP (+depth for distance) | ground-truth scene query, occlusion-checked by raycast |
| `move(f, s, t, climb)` | unit commands in [-1, 1], scaled by the **profile** | Twist on `cmd_vel` | force/velocity on the physics body |
| `grasp(closed)` | bool | gripper action/topic | (returns with the contact engine — see below) |

**Verbs** — handle-side, implemented once in `behaviors.py`, never
reimplemented per backend: `goto`, `explore`, `approach`, `locate`,
`seen`, `answer`, `say`, `remember/recall`, `state`, `think/thought`.
These compose primitives, so they are portable *by construction*. If a
backend needs special code for a verb, the backend's primitives are
wrong — fix those.

**Profiles** — per-robot scaling so `move(forward=1.0)` means the same
*relative* thing everywhere and a sane *absolute* thing per robot. The
sim uses the same numbers it expects from hardware:

| Profile | vmax m/s | yaw rad/s | climb m/s | Hardware twin |
|---|---|---|---|---|
| dog | 1.0 | 1.5 | — | Unitree Go2 (walk mode) |
| biped | 0.6 | 1.5 | — | Unitree G1 |
| drone | 1.3 | 1.5 | 1.2 | PX4-class / Mavic |
| wheeled | 0.5 | 1.2 | — | TurtleBot-class |
| arm | (workspace verbs, not cmd_vel) | | | xArm — pending contact engine |

**Capability gating** — a policy that calls a primitive the connected
robot lacks gets the existing `capabilities()` treatment (transport
already surfaces this): explicit unsupported, not silent no-op. The
deploy screen says which of your policies run on which robot.

The contract is enforced, not aspirational: `tests/test_contract.py`
runs the same policy against the arena state-shape and the transport
adapters and asserts schema equality.

## Physics: Rapier, behind the `world` wrapper

The arena's collision/dynamics move from hand-rolled AABB checks to
**rapier3d-compat** (WASM, deterministic IEEE float math, multibody
joints when we need them, actively developed — 2025 was their perf
year). Engine choice stays behind a wrapper so mujoco-wasm (DeepMind
now ships official WASM bindings, alpha) can take the contact-rich
ladders later without touching policies or levels.

`roborun/web/physics.js` — DOM-free, runs in node and the browser:

- **Static world**: walls and floor are fixed cuboid colliders.
- **Robots are kinematic character bodies** (Rapier's character
  controller): they slide along walls, step over nothing, and *push
  dynamic objects honestly*. The gait/IK stays visual-only in arena.js —
  a robot type is a character with a built-in controller (ARENA_SPEC),
  not raw joint dynamics.
- **Props with mass are dynamic bodies**: crates topple, slide with real
  friction, can be shoved by the robot or by movers. Pickup/drop stays a
  game verb (walk into it / reach the zone), but a carried crate is a
  body switched to kinematic and back — dropping it on a slope means it
  slides, because it's real.
- **Movers are kinematic bodies** on paths; they displace the robot and
  dynamic props through the same solver instead of being walls that
  teleport.
- **Sensors go through the physics world**: lidar = `castRay` ×36
  (sees walls, movers, crates — anything with a collider); `see()`
  occlusion = one raycast per candidate. One source of truth; the
  renderer never answers sensing questions.
- **Fixed timestep** (60 Hz accumulator) and a **single recorded seed**
  for all spawn jitter: the determinism the leaderboard's replay
  verification needs is architected in now, not retrofitted.

Drones get the same body with gravity off and a climb axis — altitude
is real, ring/pad contact is real.

### What does NOT change

- The websocket/Pyodide wire: `{forward, strafe, turn, climb, grip}` out,
  state snapshot in. Policies and the wasm shim don't know the engine
  changed.
- Levels: same declarative walls/props (the Python level format in
  ARENA_SPEC remains the target; the JS level table is the interim).
- Recording/sealing: unchanged, still MCAP + merkle + (native) RFC 3161.

### The arm comes back on physics or not at all

The previous arm faked grasping (centered-and-descend magnet with
collision garnish) and died of it. The hand ladder returns only on a
real contact engine (mujoco-wasm now that DeepMind ships official
bindings, or Rapier multibody + convex fingers if it proves sufficient),
with `grasp()` meaning friction closure — ARENA_SPEC already says the
hand ladder exists *because* of contact fidelity. Until then the arena
ships three honest types rather than four where one is a lie.

## Design constraints (why the sim works this way)

These are product positions, stated as constraints so they survive
contact with feature requests:

1. **Nothing installed before something moves.** The browser path needs
   zero install; `pip install ros-agent` is the whole local story.
   Time-to-first-motion is the metric every feature serves.
2. **Prototyping must not be hardware-gated.** The sim is a transport
   backend behind the same handle — the policy file works before the
   robot ships, and unchanged after it does.
3. **One file, no manifests.** A policy is one Arduino-shaped Python
   file; a level is one Python file. No blueprint/wiring layer between
   a developer and a moving robot.
4. **The sim is the front door, not an extra.** The hosted arena is the
   first-run experience; MuJoCo/Isaac/Gazebo are optional high-fidelity
   add-ons through the same ROS transport, never prerequisites.

## Build order (dependency order, no dates)

1. `physics.js` wrapper + node smoke test (`scripts/e2e_physics.mjs`):
   world build, character slide, crate push, lidar raycast, drone climb,
   determinism (same seed ⇒ identical state stream).
2. arena.js on physics.js: bodies/colliders from the level table, meshes
   become pure visuals, sensors via physics raycasts.
3. Contract tests: schema equality across arena snapshot and transport
   adapters; profile table shared between sim and real backends.
4. Interaction levels that need real physics (push-the-cube, blocked
   door, crate stacking) — the levels the magnet arm could never host.
5. Hand ladder on the contact engine (mujoco-wasm or Rapier multibody),
   `grasp()` = friction closure, xArm twin.
