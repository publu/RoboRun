/* E2E: the browser wasm runtime, headless. Loads the real roborun modules
   into Pyodide exactly like web/wasm.js does, runs a policy for 30 ticks,
   records, seals — then the files are verified by the *native* Python
   recorder to prove browser-sealed runs are interchangeable.

   Run:
     python scripts/build_site.py
     npm i pyodide@0.26.4            # anywhere on NODE_PATH, needs network
     node scripts/e2e_wasm.mjs
     python -m roborun.recorder verify /tmp/wasm_run/<run>.mcap
   Expected: E2E WASM RUNTIME OK, then CONSISTENT (unanchored). */
import { loadPyodide } from "pyodide";
import { readFileSync, writeFileSync, mkdirSync } from "fs";

const SITE = new URL("../site", import.meta.url).pathname;
const PY_FILES = ["__init__.py", "arena.py", "behaviors.py", "sightings.py",
                  "integrity.py", "recorder.py", "anchor.py", "events.py",
                  "llm.py"];

const py = await loadPyodide();
console.log("pyodide up:", py.version);
await py.loadPackage("micropip");
try { await py.loadPackage("cryptography"); console.log("cryptography loaded (Ed25519 on)"); }
catch (e) { console.log("cryptography unavailable:", e.message); }
await py.loadPackage("zstandard");
await py.runPythonAsync('import micropip; await micropip.install("mcap", deps=False)');
console.log("mcap installed");

py.FS.mkdirTree("/app/roborun");
for (const f of PY_FILES)
  py.FS.writeFile(`/app/roborun/${f}`, readFileSync(`${SITE}/py/roborun/${f}`, "utf8"));
py.FS.writeFile("/app/shim.py", readFileSync(`${SITE}/py/shim.py`, "utf8"));
py.runPython('import sys; sys.path.insert(0, "/app"); import shim');
const shim = py.pyimport("shim");
const J = (s) => JSON.parse(s);

// the dog-census demo's shape: explore, then answer from the system ledger
const policy = `
from roborun.behaviors import behavior

@behavior(hz=10)
def player_policy(robot):
    robot.state["t"] = robot.state.get("t", 0) + 1
    robot.move(forward=0.5, turn=0.2)
    if robot.state["t"] == 20:
        doors = sum(s["distinct"] for s in robot.seen() if "door" in s["label"])
        robot.answer(str(doors))
        robot.say("counted " + str(doors) + " doors")
`;
const lp = J(shim.load_policy(policy));
if (!lp.ok) throw new Error("load_policy failed: " + lp.error);
console.log("policy loaded:", lp.name);

const rec = J(shim.record_start("arena"));
console.log("recording:", rec.run);

let lastCmd = null, answer = null;
for (let i = 0; i < 30; i++) {
  // two distinct doors seen from a moving pose — exercises sighting dedup
  const dets = [{ label: "red door", confidence: 0.95,
                  bbox: [590, 250, 690, 470], distance: 2.0 }];
  if (i > 12) dets.push({ label: "red door", confidence: 0.9,
                          bbox: [200, 250, 300, 470], distance: 3.0 });
  const r = J(shim.tick(JSON.stringify({
    detections: dets,
    lidar: Array(36).fill(4.0),
    pose: { x: i * 0.3, z: 0, y: 0, heading: 0 },
    level: { name: "dog-census", robot: "dog" },
  })));
  if (r.error) throw new Error("policy error:\n" + r.error);
  lastCmd = r.cmd;
  if (r.answer) answer = r.answer;
}
console.log("cmd flows:", JSON.stringify(lastCmd));
if (!(lastCmd.forward > 0)) throw new Error("policy command did not reach arena");
if (!answer) throw new Error("no answer submitted");
console.log("answer:", answer.text, "(distinct-door dedup through real sightings.py)");

const sg = J(shim.sightings_summary());
console.log("sightings:", sg.map((s) => `${s.label}×${s.distinct}`).join(", "));

const stop = J(shim.record_stop());
if (!stop.ok) throw new Error("record_stop failed");
console.log("sealed:", stop.seal.run, "merkle:", stop.seal.merkle_root.slice(0, 16) + "…",
            "signed:", !!stop.seal.signature, "anchor:", stop.seal.anchor.status);

// in-wasm verify (what the RUNS drawer's verify button calls)
const v = J(shim.verify(stop.mcap_path));
console.log("wasm verify:", v.state);
if (v.state === "broken") throw new Error("wasm verify broke: " + v.reason);

// export for native cross-verification
mkdirSync("/tmp/wasm_run", { recursive: true });
const base = stop.seal.run;
writeFileSync(`/tmp/wasm_run/${base}.mcap`, py.FS.readFile(stop.mcap_path));
writeFileSync(`/tmp/wasm_run/${base}.chain.jsonl`, py.FS.readFile(stop.chain_path));
writeFileSync(`/tmp/wasm_run/${base}.seal`, py.FS.readFile(stop.seal_path));
console.log(`exported to /tmp/wasm_run/${base}.*`);
console.log("E2E WASM RUNTIME OK");
