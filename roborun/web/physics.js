/* RoboRun arena physics — rapier3d-compat behind the `world` wrapper
   (docs/SIM_SPEC.md). DOM-free on purpose: the same module runs in the
   browser (import map → CDN) and in node (scripts/e2e_physics.mjs), so
   interactions are verified headlessly instead of by eyeball.

   The renderer never answers physics or sensing questions. Walls and
   floor are fixed colliders; robots are kinematic character bodies that
   slide along walls and push dynamic props; crates have mass and
   friction; movers are kinematic bodies on paths; lidar and occlusion
   are raycasts through this world. Fixed 60 Hz substeps keep stepping
   independent of render rate (and replay-deterministic). */

import RAPIER from "@dimforge/rapier3d-compat";

/* One profile per robot type — the same numbers the handle expects from
   the hardware twin, so move(forward=1.0) means the same thing in the
   sim and on the robot (SIM_SPEC contract). */
export const PROFILES = {
  dog:     { vmax: 1.0, yawRate: 1.5, radius: 0.32, height: 0.50, eyeH: 0.45 },
  biped:   { vmax: 0.6, yawRate: 1.5, radius: 0.28, height: 1.50, eyeH: 1.45 },
  wheeled: { vmax: 0.5, yawRate: 1.2, radius: 0.25, height: 0.40, eyeH: 0.30 },
  drone:   { vmax: 1.3, yawRate: 1.5, radius: 0.30, height: 0.20,
             climbRate: 1.2, fly: true, yMin: 0.4, yMax: 3.4 },
};

const H = 1 / 60;                       // fixed substep
const WALL_HALF_H = 0.8, WALL_MIN_HALF = 0.15;
const GROUP_WORLD = (0x0001 << 16) | 0xffff;   // membership 1, hits all
const GROUP_NONE = 0;                          // carried props: untouchable

let _ready = null;
export function initPhysics() { return (_ready ??= RAPIER.init({})); }

export function createWorld(def, robotType) {
  const profile = PROFILES[robotType] || PROFILES.dog;
  const world = new RAPIER.World({ x: 0, y: -9.81, z: 0 });
  world.timestep = H;

  /* ── static world ── */
  const ground = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(def.bounds * 2, 0.1, def.bounds * 2)
      .setTranslation(0, -0.1, 0).setFriction(0.8)
      .setCollisionGroups(GROUP_WORLD), ground);
  // flying profiles get full-height walls — the course stays contained
  // instead of "fly over the boundary and leave the map"
  const wallHalfH = profile.fly ? profile.yMax : WALL_HALF_H;
  for (const [x1, z1, x2, z2] of def.walls || []) {
    const hx = Math.max(Math.abs(x2 - x1) / 2, WALL_MIN_HALF);
    const hz = Math.max(Math.abs(z2 - z1) / 2, WALL_MIN_HALF);
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.fixed()
      .setTranslation((x1 + x2) / 2, wallHalfH, (z1 + z2) / 2));
    world.createCollider(RAPIER.ColliderDesc.cuboid(hx, wallHalfH, hz)
      .setCollisionGroups(GROUP_WORLD), body);
  }

  /* ── the robot: kinematic character ── */
  const standY = profile.fly ? 1.0 : profile.height / 2 + 0.01;
  const robotBody = world.createRigidBody(
    RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, standY, 0));
  const half = Math.max(profile.height / 2 - profile.radius, 0.01);
  const robotCol = world.createCollider(
    RAPIER.ColliderDesc.capsule(half, profile.radius), robotBody);
  const ctl = world.createCharacterController(0.02);
  ctl.setApplyImpulsesToDynamicBodies(true);
  ctl.setCharacterMass(40);
  let heading = 0, alt = profile.fly ? 1.0 : standY;
  let acc = 0;

  /* ── props and movers ── */
  const props = new Map();              // id -> {body, col, size}
  let carried = null;                   // id while attached
  const movers = [];

  function addCrate(id, x, z, size) {
    const body = world.createRigidBody(RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(x, size / 2 + 0.02, z)
      .setLinearDamping(0.2).setAngularDamping(0.6));
    const col = world.createCollider(
      RAPIER.ColliderDesc.cuboid(size / 2, size / 2, size / 2)
        .setFriction(0.8).setRestitution(0.05).setDensity(80)
        .setCollisionGroups(GROUP_WORLD), body);
    props.set(id, { body, col, size });
  }

  function addMover(size) {
    const body = world.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(0, 0.4, 0));
    world.createCollider(RAPIER.ColliderDesc.cuboid(size / 2, 0.4, size / 2)
      .setCollisionGroups(GROUP_WORLD), body);
    movers.push(body);
    return movers.length - 1;
  }

  function substep(cmd) {
    const cl = (v) => Math.max(-1, Math.min(1, v || 0));
    heading += cl(cmd.turn) * profile.yawRate * H;
    const f = cl(cmd.forward) * profile.vmax, s = cl(cmd.strafe) * profile.vmax;
    const c = Math.cos(heading), sn = Math.sin(heading);
    const cur = robotBody.translation();
    let dy = 0;
    if (profile.fly) {
      const targetY = Math.max(profile.yMin, Math.min(profile.yMax,
        cur.y + cl(cmd.climb) * profile.climbRate * H));
      dy = targetY - cur.y;
    }
    ctl.computeColliderMovement(robotCol,
      { x: (f * c + s * sn) * H, y: dy, z: (-f * sn + s * c) * H });
    const mv = ctl.computedMovement();
    robotBody.setNextKinematicTranslation(
      { x: cur.x + mv.x, y: cur.y + mv.y, z: cur.z + mv.z });
    if (carried !== null) {
      const p = props.get(carried);
      p.body.setNextKinematicTranslation(
        { x: cur.x + mv.x, y: profile.height + 0.1, z: cur.z + mv.z });
    }
    world.step();
    alt = robotBody.translation().y;
  }

  return {
    profile,

    /* ── robot ── */
    spawn(x, z, hdg) {
      heading = hdg || 0;
      alt = profile.fly ? 1.0 : standY;
      robotBody.setTranslation({ x, y: alt, z }, true);
      acc = 0;
    },
    step(dt, cmd) {
      for (acc += dt; acc >= H; acc -= H) substep(cmd);
    },
    pose() {
      const t = robotBody.translation();
      return { x: t.x, y: t.y, z: t.z, heading };
    },

    /* ── sensing (one source of truth: this world) ── */
    lidar(rays = 36, range = 8) {
      const t = robotBody.translation();
      const origin = { x: t.x, y: 0.45, z: t.z };
      const out = [];
      for (let i = 0; i < rays; i++) {
        const a = heading + (i / rays) * Math.PI * 2;
        const ray = new RAPIER.Ray(origin, { x: Math.cos(a), y: 0, z: -Math.sin(a) });
        const hit = world.castRay(ray, range, true,
          undefined, undefined, undefined, robotBody);
        out.push(hit ? +(hit.timeOfImpact ?? hit.toi).toFixed(2) : range);
      }
      return out;
    },
    /* nearest hit between eye and a target (skipping the target itself);
       null = clear line of sight */
    occludedAt(eye, target, targetPropId = null) {
      const d = { x: target.x - eye.x, y: target.y - eye.y, z: target.z - eye.z };
      const dist = Math.hypot(d.x, d.y, d.z);
      if (dist < 1e-6) return null;
      const ray = new RAPIER.Ray(eye,
        { x: d.x / dist, y: d.y / dist, z: d.z / dist });
      const skip = targetPropId !== null ? props.get(targetPropId)?.col : undefined;
      const hit = world.castRay(ray, dist - 0.05, true,
        undefined, undefined, skip, robotBody);
      return hit ? (hit.timeOfImpact ?? hit.toi) : null;
    },
    ahead(range = 8) {
      const t = robotBody.translation();
      const ray = new RAPIER.Ray({ x: t.x, y: 0.45, z: t.z },
        { x: Math.cos(heading), y: 0, z: -Math.sin(heading) });
      const hit = world.castRay(ray, range, true,
        undefined, undefined, undefined, robotBody);
      return hit ? (hit.timeOfImpact ?? hit.toi) : null;
    },

    /* ── props ── */
    addCrate, addMover,
    setMover(i, x, z) { movers[i]?.setNextKinematicTranslation({ x, y: 0.4, z }); },
    propPose(id) {
      const p = props.get(id);
      if (!p) return null;
      const t = p.body.translation(), r = p.body.rotation();
      const v = p.body.linvel();
      return { x: t.x, y: t.y, z: t.z, rot: r,
               speed: Math.hypot(v.x, v.y, v.z) };
    },
    attachProp(id) {
      const p = props.get(id);
      if (!p || carried !== null) return false;
      p.body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased, true);
      p.col.setCollisionGroups(GROUP_NONE);
      carried = id;
      return true;
    },
    releaseProp(at = null) {
      if (carried === null) return;
      const p = props.get(carried);
      if (at) p.body.setTranslation({ x: at.x, y: at.y ?? p.size / 2 + 0.02, z: at.z }, true);
      p.body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
      p.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      p.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
      p.col.setCollisionGroups(GROUP_WORLD);
      carried = null;
    },
    carrying() { return carried; },

    free() { world.free(); },
  };
}
