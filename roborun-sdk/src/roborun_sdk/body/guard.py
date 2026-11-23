"""The obstacle guard of a walking body: pure numpy, no robot imports, so a simulated body and a real one run the same code.

It never refuses a move. It changes the commanded twist, and it is hard safety:

  Guard.observe(depth_m, K, T_base_camera, odom, t)   once per head depth frame: an occupancy memory in the odometry frame
  Guard.check(cmd, odom, head_yaw, t, lease_blind_m=)  every command tick: the corridor stop, the blind speed cap and the
                                                        blind-travel cap, along the actual travel direction atan2(vy, vx)
  Guard.stall(commanded, measured_speed, t)            a stall: commanded to move, measured not moving
  Guard.reset() / Guard.snapshot()                     forget the map / what the guard sees now

Conventions: `odom` = (x, y, yaw) of the base in the odometry frame (z up); the base frame sits on the floor under the trunk
(+x forward, +y left); the camera frame is optical (+x right, +y down, +z forward). `K` is {'fx', 'fy', 'cx', 'cy'} (or a
3x3 matrix), pixel (u, v) centred at (u, v). `T_base_camera` is the 4x4 pose of the optical camera frame in the base frame.
Depth is metres along the optical axis; 0 or non-finite = no data. Times are seconds on one monotonic clock.

Map. A grid of `cell_m` cells within `extent_m` of the base, each free or occupied with the time it was last seen; a cell
older than `memory_s` is unknown. A cell with >= `min_points` obstacle points in one frame becomes occupied; a cell with
>= `min_points` floor points and no obstacle point becomes free; occupied wins within a frame; a newer frame overwrites an
older one. observe() takes an optional `valid` mask (False = the robot's own body); with `self_mask_reach_m` set, a masked
pixel is dropped only within that reach, so a mask can never hide an obstacle beyond it. Distances run along the heading from
the base centre to the nearest obstacle point; `clearance_m` subtracts `footprint_radius_m`.
"""
from __future__ import annotations

import math
import threading

import numpy as np

SCHEMA = 'roborun-body-guard/1'
MIN_SPEED_M_S = 0.02            # below this a twist passes unchanged: turning in place and head motion are never blocked
SELF_MASK_REACH_M = 0.6         # a sensible guard.self_mask_reach_m for the K1: its arm (shoulder 0.1-0.15 m off the axis, upper
                                # arm + forearm with the fist about 0.45 m) reaches about 0.55-0.6 m stretched out. Opt-in (the
                                # record): a grown own-silhouette (body_guard_modes' carry posture) also drops far floor pixels
                                # on purpose, so the default keeps `valid` exactly as before
_OFF, _MUL = 1 << 30, 1 << 31   # cell key = (ix + _OFF) * _MUL + (iy + _OFF): +-53,000 km at 5 cm in an int64


def _keys(ix, iy):
    return (np.asarray(ix, np.int64) + _OFF) * _MUL + (np.asarray(iy, np.int64) + _OFF)


def _cells(keys):
    keys = np.asarray(keys, np.int64)
    return keys // _MUL - _OFF, keys % _MUL - _OFF


def _intrinsics(K):
    if isinstance(K, dict):
        return float(K['fx']), float(K['fy']), float(K['cx']), float(K['cy'])
    k = np.asarray(K, float).reshape(3, 3)
    return float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])


def _odom(odom):
    if isinstance(odom, dict):
        return float(odom['x']), float(odom['y']), float(odom['yaw'])
    x, y, yaw = (float(v) for v in tuple(odom)[:3])
    return x, y, yaw


def pitch_floor(cmd, head_pitch, limits):
    """The walking pitch floor: while the commanded planar speed hypot(vx, vy) >= 0.02 m/s the head pitch target is at least
    limits['walk_min_pitch_rad'] (+pitch looks down). Returns (pitch target, clamped). The station then rate-limits the
    target at the head rate, so the floor is reached at the head rate."""
    v = math.hypot(float(cmd[0]), float(cmd[1]))
    floor = float(limits['walk_min_pitch_rad'])
    if v >= MIN_SPEED_M_S and float(head_pitch) < floor:
        return floor, True
    return float(head_pitch), False


class Guard:
    """See the module docstring. guard = the profile's guard settings, limits = its limits, station = {deadman_s}. Thread-safe: a depth
    thread calls observe() while the player calls check()."""

    def __init__(self, guard: dict, limits: dict, station: dict):
        g = dict(guard)
        if g.get('schema', SCHEMA) != SCHEMA:
            raise ValueError(f"guard schema {g.get('schema')!r} is not {SCHEMA}")
        self.cell_m = float(g['cell_m'])
        self.extent_m = float(g['extent_m'])
        self.memory_s = float(g['memory_s'])
        self.floor_band_m = float(g['floor_band_m'])
        self.obstacle_z_m = tuple(float(v) for v in g['obstacle_z_m'])
        self.self_radius_m = float(g['self_radius_m'])
        reach = g.get('self_mask_reach_m')
        self.self_mask_reach_m = None if reach is None else float(reach)     # None: a `valid` mask drops at any distance
        self.footprint_radius_m = float(g['footprint_radius_m'])
        self.depth_range_m = tuple(float(v) for v in g['depth_range_m'])
        self.depth_max_age_s = float(g['depth_max_age_s'])
        self.min_points = int(g['min_points'])
        self.pixel_stride = max(1, int(g['pixel_stride']))
        self.unknown_blind_fraction = float(g['unknown_blind_fraction'])
        self.stall_command_m_s = float(g['stall_command_m_s'])
        self.stall_measured_m_s = float(g['stall_measured_m_s'])
        self.stall_s = float(g['stall_s'])
        self.speed_window_s = float(g['speed_window_s'])
        self.proximity_stop_m = float(limits['proximity_stop_m'])
        self.corridor_halfwidth_m = float(limits['corridor_halfwidth_m'])
        self.blind_v_m_s = float(limits['blind_v_m_s'])
        self.blind_travel_max_m = float(limits['blind_travel_max_m'])
        self.stop_decel_m_s2 = float(limits['stop_decel_m_s2'])
        self.deadman_s = float(station['deadman_s'])
        # Corridor cells nearer than this (along the heading, from the base centre) are below the head camera's view from
        # here: they never count toward 'cannot see the way' (an obstacle there is still stopped for once it was seen from
        # farther away). Set by the caller from the camera's geometry (player.Watch); 0 = every cell counts.
        self.unseen_near_m = 0.0
        self._lock = threading.Lock()
        self.reset()

    # -- state -------------------------------------------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the map and the stall timer ."""
        with self._lock:
            self._key = np.zeros(0, np.int64)          # sorted cell keys
            self._occ = np.zeros(0, bool)              # occupied (else free)
            self._seen = np.zeros(0, float)            # time the cell was last seen
            self._box = np.zeros((0, 4), float)        # occupied cells: xmin, ymin, xmax, ymax of their points (odom)
            self._depth_t = None                       # time of the newest depth frame
            self._now = None                           # newest time any call brought
            self._odom = None                          # base pose at the newest observe() / check()
            self._heading = 0.0                        # travel heading (base frame) of the last translating check
            self._last = {'proximity_stop': False, 'blind': False}
            self._stall_since = None
            self._stalled = False

    def _tick(self, t):
        self._now = t if self._now is None else max(self._now, t)

    def corridor_length(self, v: float) -> float:
        """L = proximity_stop_m + v*deadman_s + v^2/(2*stop_decel_m_s2): the corridor beyond the footprint at speed v."""
        v = abs(float(v))
        return max(0.0, self.proximity_stop_m) + v * self.deadman_s + v * v / (2.0 * self.stop_decel_m_s2)

    # -- observe -----------------------------------------------------------------------------------------------------
    def observe(self, depth_m, K, T_base_camera, odom, t, *, valid=None) -> dict:
        """Add one head depth frame to the memory. Returns {'points' (valid depth samples used), 'occupied', 'free'
        (cells this frame marked), 'self' (samples dropped as the robot's own body: the cylinder and the mask), 'masked'
        (samples `valid` dropped), 'mask_kept' (samples `valid` marked but beyond self_mask_reach_m: kept), 't'}."""
        fx, fy, cx, cy = _intrinsics(K)
        T = np.asarray(T_base_camera, float).reshape(4, 4)
        x0, y0, yaw = _odom(odom)
        t = float(t)
        s = self.pixel_stride
        depth = np.asarray(depth_m, np.float32)
        d = depth[::s, ::s].astype(np.float64)
        v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        lo, hi = self.depth_range_m
        ok = np.isfinite(d) & (d >= lo) & (d <= hi)
        z = d[ok]
        p = np.column_stack([(u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z]) @ T[:3, :3].T + T[:3, 3]   # base frame
        r = np.hypot(p[:, 0], p[:, 1])
        z_lo, z_hi = self.obstacle_z_m
        if valid is not None:                                  # the self-mask (bounded by the record's reach, if it sets one)
            marked = ~np.asarray(valid, bool)[::s, ::s][ok]
            masked = marked if self.self_mask_reach_m is None else marked & (r <= self.self_mask_reach_m)
            kept = int((marked & ~masked).sum())
        else:
            masked, kept = np.zeros(len(r), bool), 0
        own = ((r < self.self_radius_m) & (p[:, 2] < z_hi)) | masked     # arms, hands, own legs (and the masked body)
        near = (r <= self.extent_m) & ~own
        floor = near & (np.abs(p[:, 2]) <= self.floor_band_m)
        obstacle = near & (p[:, 2] > z_lo) & (p[:, 2] < z_hi)  # higher points cannot touch the body: ignored
        c, sn = math.cos(yaw), math.sin(yaw)
        qx = x0 + c * p[:, 0] - sn * p[:, 1]
        qy = y0 + sn * p[:, 0] + c * p[:, 1]
        key = _keys(np.floor(qx / self.cell_m), np.floor(qy / self.cell_m))

        ok_key, ok_count = np.unique(key[obstacle], return_counts=True)
        dense = ok_count >= self.min_points
        occ_key = ok_key[dense]
        if occ_key.size:                                       # each occupied cell keeps the box of its points
            order = np.argsort(key[obstacle], kind='stable')
            ox, oy = qx[obstacle][order], qy[obstacle][order]
            starts = np.concatenate([[0], np.cumsum(ok_count)[:-1]])
            box = np.column_stack([np.minimum.reduceat(ox, starts), np.minimum.reduceat(oy, starts),
                                   np.maximum.reduceat(ox, starts), np.maximum.reduceat(oy, starts)])[dense]
        else:
            box = np.zeros((0, 4))
        fl_key, fl_count = np.unique(key[floor], return_counts=True)
        free_key = fl_key[(fl_count >= self.min_points) & ~np.isin(fl_key, ok_key)]   # no obstacle point at all

        with self._lock:
            keys = np.concatenate([self._key, occ_key, free_key])
            occ = np.concatenate([self._occ, np.ones(occ_key.size, bool), np.zeros(free_key.size, bool)])
            seen = np.concatenate([self._seen, np.full(occ_key.size + free_key.size, t)])
            boxes = np.concatenate([self._box, box, np.full((free_key.size, 4), np.nan)])
            newest = len(keys) - 1 - np.unique(keys[::-1], return_index=True)[1]    # the newest entry of each cell wins
            keep = newest[seen[newest] >= t - self.memory_s]
            ix, iy = _cells(keys[keep])
            keep = keep[np.hypot((ix + 0.5) * self.cell_m - x0, (iy + 0.5) * self.cell_m - y0) <= self.extent_m]
            self._key, self._occ, self._seen, self._box = keys[keep], occ[keep], seen[keep], boxes[keep]
            self._depth_t = t if self._depth_t is None else max(self._depth_t, t)
            self._tick(t)
            self._odom = (x0, y0, yaw)
        return {'points': int(ok.sum() - masked.sum()), 'occupied': int(occ_key.size), 'free': int(free_key.size),
                'self': int(own.sum()), 'masked': int(masked.sum()), 'mask_kept': kept, 't': t}

    # -- queries -----------------------------------------------------------------------------------------------------
    def _corridor(self, x0, y0, phi, length):
        """Corridor cells along odom heading phi: centres at along-heading distance in (footprint, footprint + length]
        and within corridor_halfwidth_m across. Returns (keys, along)."""
        r0, r1, hw, cm = self.footprint_radius_m, self.footprint_radius_m + length, self.corridor_halfwidth_m, self.cell_m
        ux, uy = math.cos(phi), math.sin(phi)
        xs = [x0 + a * ux - b * uy for a in (r0, r1) for b in (-hw, hw)]
        ys = [y0 + a * uy + b * ux for a in (r0, r1) for b in (-hw, hw)]
        ix = np.arange(math.floor(min(xs) / cm) - 1, math.floor(max(xs) / cm) + 2)
        iy = np.arange(math.floor(min(ys) / cm) - 1, math.floor(max(ys) / cm) + 2)
        gx, gy = (g.ravel() for g in np.meshgrid(ix, iy, indexing='ij'))
        dx, dy = (gx + 0.5) * cm - x0, (gy + 0.5) * cm - y0
        along, cross = dx * ux + dy * uy, -dx * uy + dy * ux
        inside = (along > r0) & (along <= r1) & (np.abs(cross) <= hw)
        return _keys(gx[inside], gy[inside]), along[inside]

    def _lookup(self, keys, t):
        """For each key: (occupied, known, seen time, box); expired cells are unknown. The caller holds the lock."""
        m, n = len(keys), len(self._key)
        if not m or not n:
            return np.zeros(m, bool), np.zeros(m, bool), np.full(m, np.nan), np.full((m, 4), np.nan)
        idx = np.minimum(np.searchsorted(self._key, keys), n - 1)
        known = (self._key[idx] == keys) & (self._seen[idx] >= t - self.memory_s)
        return known & self._occ[idx], known, np.where(known, self._seen[idx], np.nan), self._box[idx]

    @staticmethod
    def _near_along(box, x0, y0, ux, uy):
        """Along-heading distance of the nearest corner of each box (the box of the points that occupied the cell)."""
        xs = np.stack([box[:, 0], box[:, 0], box[:, 2], box[:, 2]], 1) - x0
        ys = np.stack([box[:, 1], box[:, 3], box[:, 1], box[:, 3]], 1) - y0
        return (xs * ux + ys * uy).min(1)

    def proximity(self, odom, heading_rad=0.0, *, t=None, max_range=None) -> dict:
        """Nearest remembered obstacle along `heading_rad` (base frame, CCW+): occupied cells within corridor_halfwidth_m
        across, beyond the footprint and up to `max_range` (default extent_m) from the base centre. Returns
        {'distance_m', 'clearance_m', 'source' depth|memory}, None values when nothing is known there. Informational: the
        stop rule is check()."""
        none = {'distance_m': None, 'clearance_m': None, 'source': None}
        x0, y0, yaw = _odom(odom)
        phi = yaw + float(heading_rad)
        ux, uy = math.cos(phi), math.sin(phi)
        rng = self.extent_m if max_range is None else float(max_range)
        with self._lock:
            now = self._now if t is None else float(t)
            if now is None or not len(self._key):
                return none
            live = self._occ & (self._seen >= now - self.memory_s)
            if not live.any():
                return none
            ix, iy = _cells(self._key[live])
            dx, dy = (ix + 0.5) * self.cell_m - x0, (iy + 0.5) * self.cell_m - y0
            along, cross = dx * ux + dy * uy, -dx * uy + dy * ux
            near = self._near_along(self._box[live], x0, y0, ux, uy)
            seen = self._seen[live]
            depth_t = self._depth_t
        pick = (np.abs(cross) <= self.corridor_halfwidth_m) & (along > self.footprint_radius_m) & (near <= rng)
        if not pick.any():
            return none
        k = int(np.argmin(np.where(pick, near, np.inf)))
        dist = float(near[k])
        return {'distance_m': round(dist, 4), 'clearance_m': round(dist - self.footprint_radius_m, 4),
                'source': 'depth' if depth_t is not None and seen[k] >= depth_t else 'memory'}

    def check(self, cmd, odom, head_yaw, t, *, lease_blind_m) -> tuple:
        """Guard one commanded twist (vx, vy, vyaw[, ...]); returns ((vx, vy, vyaw), events). Events (dicts with 'event'):
        proximity_stop {distance_m, clearance_m, source depth|memory, heading_rad}: vx = vy = 0, vyaw unchanged;
        blind {reason unknown|stale, unknown_fraction, depth_age_s, speed_m_s, capped_m_s}: (vx, vy) scaled to blind_v_m_s;
        blind_stop {blind_travel_m}: lease_blind_m reached blind_travel_max_m, translation zero for the rest of the lease.
        Head motion is never touched (the caller keeps its head target)."""
        vx, vy, vyaw = (float(c) for c in tuple(cmd)[:3])
        x0, y0, yaw = _odom(odom)
        t = float(t)
        v = math.hypot(vx, vy)
        events = []
        if v < MIN_SPEED_M_S:
            with self._lock:
                self._tick(t)
                self._odom = (x0, y0, yaw)
                self._last = {'proximity_stop': False, 'blind': False}
            return (vx, vy, vyaw), events
        theta = math.atan2(vy, vx)
        common = {'heading_rad': round(theta, 4), 'head_yaw': round(float(head_yaw), 4), 'speed_m_s': round(v, 4), 't': t}
        if float(lease_blind_m) >= self.blind_travel_max_m:
            events.append(dict(common, event='blind_stop', blind_travel_m=round(float(lease_blind_m), 4)))
            with self._lock:
                self._tick(t)
                self._odom, self._heading = (x0, y0, yaw), theta
                self._last = {'proximity_stop': False, 'blind': True}
            return (0.0, 0.0, vyaw), events
        length = self.corridor_length(v)
        phi = yaw + theta
        keys, along = self._corridor(x0, y0, phi, length)
        with self._lock:
            self._tick(t)
            occ, known, seen, box = self._lookup(keys, t)
            depth_t = self._depth_t
            self._odom, self._heading = (x0, y0, yaw), theta
        visible = np.asarray(along) >= self.unseen_near_m            # the floor this close is below the camera's view
        if occ.any():
            near = self._near_along(box[occ], x0, y0, math.cos(phi), math.sin(phi))
            k = int(np.argmin(near))
            dist = float(near[k])
            events.append(dict(common, event='proximity_stop', distance_m=round(dist, 4),
                               clearance_m=round(dist - self.footprint_radius_m, 4), corridor_m=round(length, 4),
                               source='depth' if depth_t is not None and seen[occ][k] >= depth_t else 'memory'))
            with self._lock:
                self._last = {'proximity_stop': True, 'blind': False}
            return (0.0, 0.0, vyaw), events
        judged = known[visible] if visible.any() else known
        unknown = float(1.0 - judged.mean()) if len(judged) else 1.0
        age = None if depth_t is None else t - depth_t
        stale = age is None or age > self.depth_max_age_s
        blind = stale or unknown > self.unknown_blind_fraction
        if blind:
            scale = min(1.0, self.blind_v_m_s / v)
            events.append(dict(common, event='blind', reason='stale' if stale else 'unknown',
                               unknown_fraction=round(unknown, 4), depth_age_s=None if age is None else round(age, 4),
                               capped_m_s=round(v * scale, 4)))
            vx, vy = vx * scale, vy * scale
        with self._lock:
            self._last = {'proximity_stop': False, 'blind': blind}
        return (vx, vy, vyaw), events

    def stall(self, commanded, measured_speed, t) -> bool:
        """True once the commanded planar speed has been >= stall_command_m_s while the odometry speed over speed_window_s
        (measured_speed, from the caller's window) stayed < stall_measured_m_s for >= stall_s. Call it at a steady rate."""
        v = math.hypot(float(commanded[0]), float(commanded[1]))
        t = float(t)
        with self._lock:
            self._tick(t)
            if v >= self.stall_command_m_s and float(measured_speed) < self.stall_measured_m_s:
                if self._stall_since is None:
                    self._stall_since = t
                self._stalled = t - self._stall_since >= self.stall_s
            else:
                self._stall_since, self._stalled = None, False
            return self._stalled

    def depth_age(self, t=None):
        """Seconds since the newest depth frame at time t (default: the newest time seen), None before any frame. The
        guard's stale rule is depth_age > depth_max_age_s (check() then caps at blind_v_m_s)."""
        with self._lock:
            now = self._now if t is None else float(t)
            depth_t = self._depth_t
        if depth_t is None or now is None:
            return None
        return max(0.0, now - depth_t)

    def snapshot(self, t=None) -> dict:
        """/status health.guard: proximity along the last translating check's heading (forward before any), the last
        check's flags, the stall flag, the number of remembered (unexpired) cells, and the depth's age with the stale bound
        (depth_age_s > depth_max_age_s is the stale rule): a stream reads them to start a leg on fresh depth."""
        with self._lock:
            now = self._now if t is None else float(t)
            odom, heading, last, stalled = self._odom, self._heading, dict(self._last), self._stalled
            cells = int(np.count_nonzero(self._seen >= now - self.memory_s)) if now is not None else 0
        prox = self.proximity(odom, heading, t=now)['distance_m'] if odom is not None else None
        age = self.depth_age(now)
        return {'proximity_m': prox, 'proximity_stop': bool(last['proximity_stop']), 'blind': bool(last['blind']),
                'stalled': bool(stalled), 'memory_cells': cells,
                'depth_age_s': None if age is None else round(age, 3), 'depth_max_age_s': self.depth_max_age_s}
