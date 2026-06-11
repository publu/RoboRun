"""Mission compiler — plain language in, a runnable policy out.

The VLA-shaped front door: the player writes what they want in words
("move around, look for doors, remember where they are, then answer the
count"), and the smart tier compiles it into an @behavior policy against
the handle API. The generated code is returned to the editor — visible,
editable, hot-reloadable — so language is the interface and code stays
the artifact (and the thing the leaderboard judges).

Provider-agnostic like everything else: whatever ROBORUN_MODEL_SMART
resolves to (Anthropic / OpenAI / Gemini / local Ollama).
"""
from __future__ import annotations

from typing import Any

HANDLE_API = """\
The policy is a function decorated with @behavior(hz=10) taking `robot`,
called 10x/second. It must never block. The handle:

  robot.pose() -> {"x", "z", "heading"} | None   (drones also get "y")
  robot.lidar() -> [36 floats, meters], [0] = straight ahead, CCW
  robot.see(label=None) -> [Thing], Thing: .label .conf .cx .cy .w .h .dist
      .cx in [0,1]; bearing_rad = (0.5 - cx) * 1.323
      world point of a sighting:
        a = pose["heading"] + (0.5 - t.cx) * 1.323
        wx = pose["x"] + cos(a) * t.dist;  wz = pose["z"] - sin(a) * t.dist
  robot.seen(label=None) -> automatic sighting memory: [{"label", "count",
      "distinct" (deduped object count), "locations" [(x,z)...]}] — use this
      for "how many X did I see" questions; never hand-roll a ledger
  robot.move(forward=0, strafe=0, turn=0, climb=0)   (clamped; climb = drones)
  robot.goto(x, z, tol=0.45) -> True when arrived (steers one tick)
  robot.frontier(prefer="near") -> (x, z) | None
      WHERE IS NEW SPACE? The edge of what the robot hasn't seen; None
      once everything reachable is seen (that is your "done").
      prefer="near" sweeps systematically, prefer="far" pushes deep.
  robot.route(x, z) -> (wx, wz) | None   next waypoint toward (x,z) through
      space already seen to be clear (walls inflated); None = no known path
  robot.mapped() -> int   cells of spatial memory so far
      the explore loop is yours:
        t = robot.frontier()
        if t: robot.goto(*(robot.route(*t) or t))
        else: ...everything seen — act on it...
  robot.locate(thing) -> (x, z) world position of a sighting, or None
  robot.approach(thing, tol=0.45) -> locate + goto; True when arrived
  robot.stop() / robot.say(text) / robot.log(msg)
  robot.answer(text)             (submit the chamber's answer)
  robot.remember(k, v) / robot.recall(k)   (persistent)
  robot.state                    (dict, survives ticks — put ALL loop state here)
  robot.think(prompt) / robot.thought()    (async LLM, safe at 10 Hz)

Rules: keep all state in robot.state (the function re-runs every tick);
never call time.sleep; never loop forever inside one tick.
"""

EXAMPLE = '''\
# mission: reach the beacon, dodging walls
from roborun.behaviors import behavior

@behavior(hz=10)
def player_policy(robot):
    beacon = robot.see("beacon")
    scan = robot.lidar()
    ahead = min(scan[:2] + scan[-2:]) if scan else 8
    if ahead < 1.0:
        robot.move(turn=1.2)
    elif beacon:
        robot.move(forward=0.9, turn=-1.5 * (beacon[0].cx - 0.5))
    else:
        robot.move(forward=0.5, turn=0.6)
'''


def compile_mission(mission: str, context: str = "") -> dict[str, Any]:
    """Language -> policy source. Returns {ok, source} or {ok: False, error}."""
    from roborun import llm
    system = (
        "You compile robot mission descriptions into Python policy code for "
        "RoboRun. Output ONLY a Python code block, nothing else. The function "
        "must be named player_policy.\n\n" + HANDLE_API +
        "\nExample of the expected shape:\n```python\n" + EXAMPLE + "```")
    prompt = (f"Level context: {context}\n\n" if context else "") + \
        f"Mission (compile this into a policy):\n{mission}"
    try:
        raw = llm.complete(prompt, system=system, tier="smart", max_tokens=1800)
    except Exception as exc:
        from roborun.llm import resolve
        provider, model = resolve("smart")
        return {"ok": False,
                "error": f"LLM unavailable ({provider}:{model} — {exc}). Set "
                         "ANTHROPIC_API_KEY / OPENAI_API_KEY / GEMINI_API_KEY "
                         "(or run Ollama) and restart roborun."}
    source = _extract_code(raw)
    if not source:
        return {"ok": False, "error": "model returned no code block"}
    if "@behavior" not in source or "def player_policy" not in source:
        return {"ok": False, "error": "generated code is not a player_policy behavior"}
    try:
        compile(source, "mission.py", "exec")
    except SyntaxError as exc:
        return {"ok": False, "error": f"generated code has a syntax error: {exc}"}
    header = "# mission: " + " ".join(mission.split())[:160] + "\n"
    if not source.startswith("# mission:"):
        source = header + source
    return {"ok": True, "source": source}


def _extract_code(raw: str) -> str | None:
    text = raw.strip()
    if "```" in text:
        parts = text.split("```")
        for part in parts[1:]:
            body = part
            if body.startswith("python"):
                body = body[6:]
            body = body.strip("\n")
            if "def " in body:
                return body
        return None
    return text if "def " in text else None
