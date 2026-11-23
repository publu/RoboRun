"""Plays a manifest.locomotion.v1 timeline on a BaseAdapter and a HeadAdapter.

Row j covers [t_j, t_j + 1/rate): its vx, vy, vyaw are the base's mean velocity over the row, its head_pitch and head_yaw are
the head's targets at the row's end. The player sends commands at the profile's command rate (50 Hz): the velocity moves
toward each row's within the profile's acceleration limits (no step at a row boundary), the head is interpolated across the
row, and every value is clamped to the profile's limits. After the last row the base is brought to zero the same way, then
stopped. `stop` (a threading.Event) ends a run at the next tick (20 ms): base.stop() at once, head held where it is.
"""
from __future__ import annotations

import math
import threading
import time

from . import profiles


def _shape(current, want, step):
    return [c + max(-s, min(s, w - c)) for c, w, s in zip(current, want, step)]


def play(rows, base, head, profile_id, *, rate_hz=10., stop=None, clock=time.monotonic, sleep=time.sleep, on_tick=None, watch=None):
    """Runs the rows. Returns {played_s, rows, stopped, end: base state, guard_stop?}. Raises nothing for a Stop: `stopped`
    says so. With a Watch every command goes through the obstacle guard, and a guard stop ends the run (guard_stop)."""
    p = profiles.load(profile_id)
    lim, motion, dof = p['robot']['limits'], p['motion'], p['robot']['head_dof']
    hz = float(motion.get('command_hz') or 50.)
    dt = 1. / hz
    accel = list(motion['accel_m_s2']) + [motion['accel_yaw_rad_s2']]
    step = [a * dt for a in accel]
    row_s = 1. / float(rate_hz or 10.)
    rows = [r for r in rows if isinstance(r, dict)]
    clean = lambda r: [profiles.clamp(r.get('vx') or 0., lim['vx_m_s']), profiles.clamp(r.get('vy') or 0., lim['vy_m_s']),
                       profiles.clamp(r.get('vyaw') or 0., lim['vyaw_rad_s'])]
    start_head = head.state() if dof else dict(pitch=0., yaw=0.)
    heads = [(float(start_head['pitch']), float(start_head['yaw']))]
    for r in rows:
        pitch = r.get('head_pitch'); yaw = r.get('head_yaw')
        pitch = profiles.clamp(pitch, lim['head_pitch_rad']) if dof >= 1 and pitch is not None else heads[-1][0]
        yaw = profiles.clamp(yaw, lim['head_yaw_rad']) if dof >= 2 and yaw is not None else heads[-1][1]
        heads.append((pitch, yaw))
    vel = [0., 0., 0.]
    t0 = clock(); ticks = 0; stopped = False; guard_stop = None
    head_yaw = 0.
    total = len(rows) * row_s
    try:
        while True:
            if stop is not None and stop.is_set():
                stopped = True
                break
            t = clock() - t0
            j = int(t / row_s)
            if j < len(rows):
                vel = _shape(vel, clean(rows[j]), step)
                if dof:
                    a = min(1., (t - j * row_s) / row_s)
                    (p0, y0), (p1, y1) = heads[j], heads[j + 1]
                    head_yaw = y0 + (y1 - y0) * a
                    head.command(p0 + (p1 - p0) * a, head_yaw)
            else:                                            # after the last row: down to zero within the same limits
                vel = _shape(vel, [0., 0., 0.], step)
                if all(abs(v) < 1e-9 for v in vel):
                    base.command(0., 0., 0.)
                    break
            send = vel
            if watch is not None:
                send, guard_stop = watch.check(vel, head_yaw)
                if guard_stop is not None:
                    stopped = True
                    break
            base.command(*send)
            ticks += 1
            if on_tick:
                on_tick(t, vel)
            nxt = t0 + ticks * dt
            delay = nxt - clock()
            if delay > 0:
                if stop is not None:
                    if stop.wait(delay):
                        stopped = True
                        break
                else:
                    sleep(delay)
    finally:
        base.stop()                                          # a run always ends stopped, whatever happened
        if stopped and dof:
            h = head.state()
            head.command(h['pitch'], h['yaw'])               # the head stays where it is
    out = dict(played_s=round(min(clock() - t0, total + 5.), 3), rows=len(rows), stopped=stopped, end=base.state())
    if guard_stop is not None:
        out['guard_stop'] = guard_stop
    return out


# ------------------------------------------------------------------------------------------------ settle
SETTLE_TOL_M = 0.02              # where the plan ends, within this much
SETTLE_TOL_RAD = 0.026           # and 1.5°
SETTLE_GAIN = 2.0                # 1/s: the correction speed per metre (or radian) still to go
SETTLE_MAX_M = 0.5               # a bigger miss is not corrected blind (a push, a slip, the guard): it is reported
SETTLE_MAX_RAD = 0.8
SETTLE_SPEED = (0.15, 0.15, 0.5)   # at most this fast while correcting (vx, vy m/s; vyaw rad/s)
SETTLE_S = 4.0


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def planned_end(rows, rate_hz=10.):
    """Where the rows end, in the frame of the base at their start: (x, y, yaw)."""
    dt = 1. / float(rate_hz or 10.)
    x = y = yaw = 0.
    for r in rows:
        if not isinstance(r, dict):
            continue
        vx, vy, w = float(r.get('vx') or 0.), float(r.get('vy') or 0.), float(r.get('vyaw') or 0.)
        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
        yaw += w * dt
    return x, y, yaw


def _error(start, goal, now):
    """What is still to go to `goal` (in start's frame), in the base's current frame: (ex, ey, eyaw)."""
    c0, s0 = math.cos(start['yaw']), math.sin(start['yaw'])
    gx, gy = start['x'] + c0 * goal[0] - s0 * goal[1], start['y'] + s0 * goal[0] + c0 * goal[1]
    dx, dy = gx - now['x'], gy - now['y']
    c, s = math.cos(now['yaw']), math.sin(now['yaw'])
    return c * dx + s * dy, -s * dx + c * dy, _wrap(start['yaw'] + goal[2] - now['yaw'])


def settle(goal, start, base, profile_id, *, stop=None, clock=time.monotonic, sleep=time.sleep, watch=None):
    """After the rows: close what the body did not reach of the plan's end, measured on its own odometry, slowly and within the
    profile's limits. A miss larger than SETTLE_MAX_M / SETTLE_MAX_RAD is reported, never driven at. Returns {settled,
    error_m, error_deg, settle_s, why?}; the base ends stopped."""
    p = profiles.load(profile_id)
    lim = p['robot']['limits']
    dt = 1. / float(p['motion'].get('command_hz') or 50.)
    t0 = clock()
    out = dict(settled=False)
    try:
        while True:
            now = base.state()
            ex, ey, eyaw = _error(start, goal, now)
            out.update(error_m=round(math.hypot(ex, ey), 4), error_deg=round(math.degrees(eyaw), 2))
            if math.hypot(ex, ey) <= SETTLE_TOL_M and abs(eyaw) <= SETTLE_TOL_RAD:
                out['settled'] = True
                break
            if math.hypot(ex, ey) > SETTLE_MAX_M or abs(eyaw) > SETTLE_MAX_RAD:
                out['why'] = 'too_far'
                break
            if now.get('blocked'):
                out['why'] = 'blocked'
                break
            if (stop is not None and stop.is_set()) or clock() - t0 > SETTLE_S:
                out['why'] = 'stopped' if stop is not None and stop.is_set() else 'timeout'
                break
            want = [SETTLE_GAIN * ex, SETTLE_GAIN * ey if lim['vy_m_s'][1] > 0 else 0., SETTLE_GAIN * eyaw]
            cmd = [max(-cap, min(cap, w)) for w, cap in zip(want, SETTLE_SPEED)]
            floor = float(lim.get('deadband_m_s') or 0.)           # a base that ignores tiny commands still gets there
            lin = math.hypot(cmd[0], cmd[1])
            if floor and 0 < lin < floor * 1.2 and math.hypot(ex, ey) > SETTLE_TOL_M:
                cmd[0], cmd[1] = cmd[0] * floor * 1.2 / lin, cmd[1] * floor * 1.2 / lin
            cmd = [profiles.clamp(cmd[0], lim['vx_m_s']), profiles.clamp(cmd[1], lim['vy_m_s']), profiles.clamp(cmd[2], lim['vyaw_rad_s'])]
            if watch is not None:                           # the correction goes through the obstacle guard too
                cmd, guard_stop = watch.check(cmd)
                if guard_stop is not None:
                    out.update(why='blocked', guard_stop=guard_stop)
                    break
            base.command(*cmd)
            if stop is not None:
                if stop.wait(dt):
                    out['why'] = 'stopped'
                    break
            else:
                sleep(dt)
    finally:
        base.stop()
    out['settle_s'] = round(clock() - t0, 3)
    return out


# ------------------------------------------------------------------------------------------------ the obstacle guard
DEPTH_HZ = 10.0                  # head depth frames into the guard's map while a move runs
ODOM_HZ = 20.0


class Watch:
    """The obstacle guard around one move (guard.py): head depth into its map, odometry, and every command through check().
    A proximity stop or the blind-travel cap stops the move: `stop_event` says which, and the base is never driven on (and
    never backs off by itself). A head without depth still moves, slowly and at most blind_travel_max_m per move."""

    def __init__(self, profile_id, base, head, *, clock=time.monotonic):
        from .guard import Guard
        p = profiles.load(profile_id)
        g = dict(p.get('guard') or {})
        motion = p['motion']
        limits = dict(proximity_stop_m=g.pop('proximity_stop_m', .35), corridor_halfwidth_m=g.pop('corridor_halfwidth_m', .4),
                      blind_v_m_s=g.pop('blind_v_m_s', .15), blind_travel_max_m=g.pop('blind_travel_max_m', .4),
                      stop_decel_m_s2=motion['stop_decel_m_s2'])
        self.guard = Guard(g, limits, dict(deadman_s=motion['deadman_s']))
        self.base, self.head, self.clock = base, head, clock
        self.has_depth = None
        self.odom = None
        self.blind_m = 0.
        self._blind_from = None
        self.stop_event = None
        self.events = dict(proximity_stop=0, blind=0, blind_stop=0)
        self.blind_why = {}                       # why the guard could not see the way: stale/unknown counts, worst values
        self._closed = threading.Event()
        self._threads = []

    def _depth_once(self):
        try:
            d = self.head.depth()
        except Exception:
            d = None
        if d is None:
            return False
        self.guard.observe(d['depth_m'], d['K'], d['T_base_camera'], d['odom'], self.clock())
        near = nearest_floor_seen(d['K'], d['T_base_camera'], d['depth_m'].shape)
        if near is not None:
            self.guard.unseen_near_m = max(0., near - self.guard.cell_m)
        return True

    def _odom_once(self):
        s = self.base.state()
        self.odom = dict(x=float(s['x']), y=float(s['y']), yaw=float(s['yaw']))

    def start(self):
        """One depth frame and one odometry reading before the move, so it starts seeing; then both keep coming."""
        self._odom_once()
        self.has_depth = self._depth_once()
        loops = [(self._odom_once, ODOM_HZ)] + ([(self._depth_once, DEPTH_HZ)] if self.has_depth else [])
        for fn, hz in loops:
            t = threading.Thread(target=self._loop, args=(fn, hz), name='body-guard', daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def _loop(self, fn, hz):
        while not self._closed.wait(1. / hz):
            try:
                fn()
            except Exception:
                pass

    def check(self, cmd, head_yaw=0.):
        """(vx, vy, vyaw) to send, and the stop event (or None). Travel while the guard cannot see the way counts toward the
        blind-travel cap."""
        if self.stop_event is not None:
            return (0., 0., 0.), self.stop_event
        odom = self.odom
        if odom is None:
            return (0., 0., 0.), None
        if self._blind_from is not None:
            self.blind_m += math.hypot(odom['x'] - self._blind_from['x'], odom['y'] - self._blind_from['y'])
            self._blind_from = None
        out, events = self.guard.check(cmd, odom, head_yaw, self.clock(), lease_blind_m=self.blind_m)
        for e in events:
            self.events[e['event']] = self.events.get(e['event'], 0) + 1
            if e['event'] in ('proximity_stop', 'blind_stop'):
                self.stop_event = e
                return (0., 0., 0.), e
            if e['event'] == 'blind':
                self._blind_from = odom
                why = self.blind_why
                why[e.get('reason') or '?'] = why.get(e.get('reason') or '?', 0) + 1
                why['max_unknown'] = max(why.get('max_unknown', 0.), float(e.get('unknown_fraction') or 0.))
                if e.get('depth_age_s') is not None:
                    why['max_depth_age_s'] = max(why.get('max_depth_age_s', 0.), float(e['depth_age_s']))
                why['max_head_yaw'] = max(why.get('max_head_yaw', 0.), abs(float(head_yaw)))
        return out, None

    def close(self):
        self._closed.set()
        for t in self._threads:
            t.join(timeout=1.)

    def summary(self):
        out = dict(depth=bool(self.has_depth), blind_travel_m=round(self.blind_m, 3), events=dict(self.events))
        if self.blind_why:
            out['blind'] = {k: round(v, 3) if isinstance(v, float) else v for k, v in self.blind_why.items()}
        if self.stop_event is not None:
            e = self.stop_event
            out['stop'] = dict(event=e['event'], distance_m=e.get('distance_m'), clearance_m=e.get('clearance_m'),
                               blind_travel_m=e.get('blind_travel_m'))
        return out


def nearest_floor_seen(K, T_base_camera, shape):
    """How far ahead of the base centre the head camera's view reaches the floor (its bottom row, straight ahead), or None
    when the bottom of the image does not look down at the floor."""
    T = [[float(v) for v in row] for row in T_base_camera]
    fy, cy = float(K['fy']), float(K['cy'])
    v = shape[0] - 1
    ray_cam = (0., (v - cy) / fy, 1.)                       # the bottom row, the image's centre column
    ray = [sum(T[i][j] * ray_cam[j] for j in range(3)) for i in range(3)]
    if ray[2] >= -1e-6:
        return None
    s = -T[2][3] / ray[2]
    return max(0., T[0][3] + s * ray[0])


def stop_reason(stop, has_depth):
    """A guard stop in plain words."""
    if stop['event'] == 'proximity_stop':
        c = stop.get('clearance_m')
        return f'stopped: something about {max(0., c if c is not None else stop.get("distance_m") or 0.):.1f} m ahead'
    if not has_depth:
        return (f"stopped after {stop.get('blind_travel_m') or 0.:.2f} m: this body sends no depth, so it cannot see the way "
                'and moves at most that far per move')
    return f"stopped after {stop.get('blind_travel_m') or 0.:.2f} m: it could not see the way ahead"
