# RoboRun Arena — browser sim, levels, vibecoded policies

Status: design. The wedge: a **game for developers** — vibecode a policy,
beat a level, climb a leaderboard whose scores are sealed runs. The same
policy file that beats Level 3 drives a real robot, because both speak the
same `robot.see() / robot.move()` handle. The game is the top of the
adoption funnel; roborun is what you graduate into.

## The five users (and what each one runs)

| User | Sim | Install | Policy runtime |
|---|---|---|---|
| 1. Webcam tinkerer | webcam + YOLO (today's flow) | `pip install ros-agent` | local Python behaviors |
| 2. Test robots on my machine | **three.js arena** in the flight deck | `pip install ros-agent` | local Python behaviors |
| 3. Game player (viral path) | **three.js arena, hosted website** | none | in-browser (Pyodide) |
| 4. Isaac/Gazebo person | their sim, their CUDA | theirs + `[ros]` | local Python behaviors |
| 5. Real robot | hardware | `pip install ros-agent` | local Python behaviors |

One policy format across all five. The arena is not a separate product —
it's a third actuator/sensor backend behind the existing handle, next to
MuJoCo and rosbridge.

## Why three.js (decision)

- **No CUDA, no native deps, no install for user 3.** MuJoCo stays an
  optional `[sim]` extra; it stops being the default first-run experience.
- **We own the UI/UX.** The flight deck is already a browser app; the arena
  renders in the same page, same event timeline, same anchor badge.
- Physics: a deterministic JS engine (rapier.js compiled to WASM —
  cross-platform deterministic, fixed timestep) — determinism is
  load-bearing for score verification (below).

## Architecture: the sim is a transport backend

```
behaviors/*.py (10 Hz, L1)             three.js arena (browser, 60 fps)
   robot.move(f,s,t)  ──ws──▶  apply cmd_vel to robot body
   robot.see("cube")  ◀──ws──  ground-truth detections from the scene graph
```

- The arena publishes **ground-truth detections** (label, bbox, distance)
  from the scene graph — no YOLO, no GPU, zero perception latency. This is
  the "instant actions" answer: L1 policies run against snapshots at full
  rate; `robot.ask()` stays in `every=` loops and is *scored against you*
  (see scoring). The game mechanically teaches SPEED_LAYERS.md.
- Local mode (users 2, 4, 5): browser arena ↔ local `ros-agent` over the
  existing websocket; policies are real files in `behaviors/`, hot-reload
  works live mid-level.
- Hosted mode (user 3): static site; policies run **in the browser via
  Pyodide** against the same `robot` handle shim — the file is portable
  down to local mode unchanged. No server compute per player.

## MCP flow ("select the policy you vibecoded")

- Local: already works — `ros-agent-mcp`; Claude/Cursor writes
  `behaviors/level3.py`, watches the timeline, iterates.
- Hosted: the site exposes a per-session **streamable-HTTP MCP URL**
  (`https://play.../mcp/<session>`) with tools: `get_level` (goal, scene,
  scoring), `read_policy` / `write_policy`, `run_level`, `get_result`.
  Player pastes the URL into Claude/Cursor; the agent vibecodes against the
  live sim. The site is simultaneously playable by hand (built-in editor)
  for people without an MCP client.

## Levels — Python, Arduino-shaped (never JSON)

Everything a dev authors in this system is a small Python file that reads
like Arduino code: module level is `setup()`, the decorated/named hooks are
`loop()`. Policies already have this shape (`@behavior(hz=10)`). Levels get
the same treatment — a scene is *built*, dynamics are *coded*, the win
condition is a *function*, because real levels need logic (moving hazards,
buttons, doors, staged goals) and JSON can't express logic without growing
an awkward mini-language:

```python
"""Level 2 — push the cube into the glowing zone."""
NAME, TIME_LIMIT, LLM_BUDGET = "push-it", 90, 2

def build(world):                     # setup()
    world.robot(x=0, y=0)
    world.box("cube", x=2, y=1, color="orange")
    world.zone("goal", x=4, y=3, r=0.8, glow=True)
    world.bot("patroller", x=3, y=0)

def tick(world):                      # loop() — optional dynamics
    world.get("patroller").follow([(3, 0), (3, 4)], speed=0.5)

def win(world):
    return world.inside("cube", "goal", for_seconds=2.0)
```

The `world` API is the level-author's mirror of the `robot` handle: a
handful of verbs, no framework. In hosted mode levels run in the same
Pyodide runtime as policies (sandboxed); locally they're files, hot-reload
included. Levels are repos — same fork-and-install story as skills
(`ros-agent level add owner/repo`; the site loads community levels by
GitHub URL, SHA-pinned like skills so a leaderboard level can't be edited
after scores exist). Campaign sketch: L1 drive to the beacon → L2 push the
cube → L3 patrol with moving obstacles → L4 find the *described* object
(first level where `robot.ask()` earns its cost) → L5 multi-robot relay
(beacons).

## Scoring + merkleized timelines

Score = f(time, energy, collisions, **LLM calls used** — fewer is better;
good speed-layering wins). Verification is two layers, built in order:

1. **Sealed runs (exists today).** Every level attempt records into the
   MCAP recorder: commands, detections, sim state, win event. Seal = chunk
   hash chain → Merkle root → Ed25519 → RFC 3161 timestamp. A leaderboard
   entry is `(score, seal, run)` — tamper-evident, externally timestamped,
   replayable in the flight deck or Foxglove. In-browser: the recorder's
   chain/seal logic ports to JS (sha256 + ed25519 are tiny); hosted runs
   seal client-side and upload run+seal.
2. **Deterministic re-execution (the real anti-cheat, later).** Fixed
   timestep + WASM physics + seeded RNG ⇒ replaying the recorded command
   stream against the level must reproduce the recorded states. The
   leaderboard re-runs submissions headless and rejects divergence. The
   seal pins *what* was claimed; determinism proves it *happened*.
   Architect now: every nondeterminism (RNG, spawn jitter) flows from one
   recorded seed; commands are recorded with their sim-tick index.

## What this is NOT

- Not a MuJoCo replacement for dynamics fidelity — the arena is gameplay
  + portability, Isaac/Gazebo/MuJoCo remain the high-fidelity path (user 4)
  through the same ROS transport.
- Not an LLM-in-the-loop game. The LLM writes the policy; the policy plays
  the level. (An optional "ask" tool inside a level is a scored luxury.)

## Build order

1. **Arena backend in the flight deck** (local mode): rapier.js + three.js
   page, ws bridge into the existing simulator slot (`set_cmd_vel`,
   detections feed), one hardcoded level. MuJoCo demoted to `[sim]` extra.
2. **Levels + scoring + sealed attempts**: level JSON loader, win
   predicates, score HUD, auto-record each attempt via the recorder.
3. **Hosted site**: static build of the same arena + Pyodide policy
   runtime + per-session MCP endpoint; client-side seal; leaderboard
   that verifies seals.
4. **Deterministic replay verification** + community levels via GitHub.
