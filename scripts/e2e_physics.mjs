/* E2E: the arena physics, headless. Builds worlds from level-shaped
   defs and proves the interactions are real — wall sliding without
   tunneling, crates that move when shoved, lidar that sees what's
   there, drone altitude limits, carry/release, and determinism (same
   commands ⇒ bit-identical trajectories, what replay verification
   will lean on).

   Run:
     npm i @dimforge/rapier3d-compat   # anywhere on NODE_PATH
     node scripts/e2e_physics.mjs
   Expected: E2E PHYSICS OK */
import { initPhysics, createWorld, PROFILES } from "../roborun/web/physics.js";

let failures = 0;
function check(name, cond, detail = "") {
  console.log(`${cond ? "ok " : "FAIL"} ${name}${detail ? " — " + detail : ""}`);
  if (!cond) failures++;
}
const run = (w, secs, cmd) => { for (let i = 0; i < secs * 60; i++) w.step(1 / 60, cmd); };

await initPhysics();

/* ── 1. walls stop and slide, no tunneling ── */
{
  const w = createWorld({ bounds: 8, walls: [[2, -4, 2, 4]] }, "dog");
  w.spawn(0, 0, 0);                       // heading 0 = +x, wall at x=2
  run(w, 4, { forward: 1 });
  const p = w.pose();
  check("wall stops the dog", p.x > 1.2 && p.x < 2.0, `x=${p.x.toFixed(2)}`);
  w.spawn(0, 0.5, 0.3);                   // angled approach must slide along -z
  run(w, 4, { forward: 1 });
  const q = w.pose();
  check("wall slides, not sticks", q.z < -0.5 && q.x < 2.0,
        `x=${q.x.toFixed(2)} z=${q.z.toFixed(2)}`);
  w.free();
}

/* ── 2. crates are dynamic: shoving moves them, they settle ── */
{
  const w = createWorld({ bounds: 8, walls: [] }, "dog");
  w.addCrate(0, 1.5, 0, 0.5);
  w.spawn(0, 0, 0);
  run(w, 3, { forward: 1 });
  const c = w.propPose(0);
  check("shoved crate moved", c.x > 1.8, `x=${c.x.toFixed(2)}`);
  run(w, 2, {});                          // let it settle
  const s = w.propPose(0);
  check("crate settles on floor", Math.abs(s.y - 0.25) < 0.05 && s.speed < 0.05,
        `y=${s.y.toFixed(3)} v=${s.speed.toFixed(3)}`);
  w.free();
}

/* ── 3. lidar sees walls, movers, and crates at honest ranges ── */
{
  const w = createWorld({ bounds: 8, walls: [[3, -4, 3, 4]] }, "dog");
  w.addCrate(0, -2, 0, 0.5);
  const mv = w.addMover(0.9);
  w.setMover(mv, 0, -2);
  w.spawn(0, 0, 0);
  w.step(1 / 60, {});
  const scan = w.lidar();
  check("lidar: wall face ahead ~2.85m", Math.abs(scan[0] - 2.85) < 0.1, `${scan[0]}`);
  check("lidar: crate face behind ~1.75m", Math.abs(scan[18] - 1.75) < 0.1, `${scan[18]}`);
  check("lidar: mover at left ~1.55m", Math.abs(scan[9] - 1.55) < 0.2, `${scan[9]}`);
  const block = w.occludedAt({ x: 0, y: 0.45, z: 0 }, { x: 5, y: 0.45, z: 0 });
  check("occlusion: wall blocks sight", block !== null && block < 3.0, `${block}`);
  const clear = w.occludedAt({ x: 0, y: 0.45, z: 0 }, { x: -2, y: 0.25, z: 0 }, 0);
  check("occlusion: own target excluded", clear === null, `${clear}`);
  w.free();
}

/* ── 4. drone: real altitude, clamped ── */
{
  const w = createWorld({ bounds: 8, walls: [] }, "drone");
  w.spawn(0, 0, 0);
  run(w, 1, { climb: 1 });
  const up = w.pose().y;
  check("drone climbs", up > 1.5, `y=${up.toFixed(2)}`);
  run(w, 10, { climb: 1 });
  check("altitude ceiling holds", Math.abs(w.pose().y - PROFILES.drone.yMax) < 0.05,
        `y=${w.pose().y.toFixed(2)}`);
  run(w, 10, { climb: -1 });
  check("altitude floor holds", Math.abs(w.pose().y - PROFILES.drone.yMin) < 0.05,
        `y=${w.pose().y.toFixed(2)}`);
  w.free();
}

/* ── 5. carry: attach follows, release drops and settles ── */
{
  const w = createWorld({ bounds: 8, walls: [] }, "biped");
  w.addCrate(0, 0.5, 0, 0.5);
  w.spawn(0, 0, 0);
  check("attach", w.attachProp(0) === true);
  run(w, 3, { forward: 1 });
  const p = w.pose(), c = w.propPose(0);
  check("carried crate rides along", Math.hypot(c.x - p.x, c.z - p.z) < 0.1,
        `d=${Math.hypot(c.x - p.x, c.z - p.z).toFixed(3)}`);
  w.releaseProp({ x: p.x + 1, y: 1.0, z: p.z });   // set down ahead (arena's deliver path)
  run(w, 2, {});
  const s = w.propPose(0);
  check("released crate lands", Math.abs(s.y - 0.25) < 0.05 && s.speed < 0.05,
        `y=${s.y.toFixed(3)}`);
  w.free();
}

/* ── 6. determinism: same commands ⇒ identical trajectory ── */
{
  const trace = () => {
    const w = createWorld({ bounds: 8, walls: [[3, -4, 3, 4]] }, "dog");
    w.addCrate(0, 1.5, 0.1, 0.5);
    w.spawn(0, 0, 0);
    const out = [];
    for (let i = 0; i < 600; i++) {
      w.step(1 / 60, { forward: Math.sin(i / 40), turn: Math.cos(i / 60) });
      const p = w.pose(), c = w.propPose(0);
      out.push(p.x, p.z, p.heading, c.x, c.y, c.z);
    }
    w.free();
    return out;
  };
  const a = trace(), b = trace();
  check("deterministic replay", a.length === b.length && a.every((v, i) => v === b[i]));
}

console.log(failures ? `E2E PHYSICS FAILED (${failures})` : "E2E PHYSICS OK");
process.exit(failures ? 1 : 0);
