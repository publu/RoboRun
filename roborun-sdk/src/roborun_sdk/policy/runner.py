"""The policy runner: one episode of a policy on this robot, as a Roborun task.

Order, before anything moves: the provider's needs() (offline) -> the match check against this robot -> consent for a remote
provider -> start() -> warm-up (act() until the first chunk, nothing leased) -> the start-motion rule -> the lease -> push.
While it runs: a refill tick takes one fresh pose (and frames), the provider's act() runs on its own worker thread, and the
chunk is pushed with skip_s = now - the observation's time, so the station plays only the rows still due. The heartbeat runs
on its own thread at 4 Hz. Stop from anywhere ends the lease at the station; the runner sees it, stops pushing and closes the
task record. Before the lease there is nothing to end: the runner reads the station's Stop counter (hold_generation) at
admit() and throughout warm-up, and the lease carries it, so a Stop pressed while the model warms up ends the run unmoved. The provider's end_episode() and close() run afterwards on a separate thread with a 3 s cap.

Threads: the control loop (refill decisions, pushes, observe ticks, Stop detection), the worker (act, one call in flight), the
heartbeat (4 Hz) and the recorder (30 Hz pose, 10 Hz cameras).
"""
from __future__ import annotations

import json
import math
import queue
import random
import secrets
import threading
import time
import traceback
from dataclasses import dataclass, field, replace
from pathlib import Path

from . import consent as consent_mod
from .provider import (Chunk, Needs, PolicyNotStarted, PolicyRefused, ProviderError, RunSpec)
from .record import (MB, Paths, Recorder, TaskRecord, append_line, cap_text, make_room, recording_settings,
                     robot_runtime, robot_wants_dataset, runtime_config, write_episode, write_json)
from .station import Cameras, Observer, Station, StationRefused, arms_of, hold_generation
from .wording import provider_text

START_MOTION_S = 60.0          # a first chunk later than this after the start: ask before moving (never move on its own)
WAIT_READY_S = 120.0           # how long the model may warm up (--wait-ready), at most WAIT_READY_MAX_S
WAIT_READY_MAX_S = 1800.0
WARMING_RETRY_S = 30.0
STALL_LIMIT_S = 8.0            # no accepted chunk for this long: the run fails (the arm has held since the buffer ran dry)
HOLD_POLL_S = 0.25             # before the lease: how often the station's Stop counter is read while waiting for the model
MAX_CHUNK_AGE_S = 3.0          # a chunk older than this when it is ready is discarded (never above the station's 3 s)
HEARTBEAT_HZ = 4.0
LOOP_HZ = 30.0
END_CAP_S = 3.0                # end_episode() + close() run off the Stop path, at most this long
SAME_SHAPE_FAILS = 3           # this many identical wrong-shape replies in a row: the run fails
SHADOW_SECONDS = 20.0
LEASE_MARGIN_S = 5.0
MAX_SECONDS = 600.0 - LEASE_MARGIN_S
REASONS = {'supervisor_stop': 'stopped', 'action_window_finished': 'done', 'policy_finished': 'done',
           'client_disconnected': 'failed', 'policy_error': 'failed', 'station_error': 'failed'}


@dataclass
class RunResult:
    task_id: str
    status: str                       # done | stopped | failed | refused | not_started
    stop_reason: str | None
    message: str
    summary: dict = field(default_factory=dict)


@dataclass
class ShadowReport:
    run_id: str
    chunks: int
    seam_rad_max: float | None
    step_from_measured_rad_max: float | None
    joint_speed_rad_s_max: float | None
    would_refuse: dict
    path: str


class _End(Exception):
    def __init__(self, status, stop_reason, message):
        super().__init__(message)
        self.status, self.stop_reason, self.message = status, stop_reason, message


def _percentile(values, p):
    if not values:
        return None
    v = sorted(values)
    k = (len(v) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return round(v[lo] + (v[hi] - v[lo]) * (k - lo), 1)


def _config():
    """robot.json, or None when there is none (or it cannot be read, or is not an object)."""
    from .. import robot
    try:
        got = json.loads(robot.CONFIG.read_text())
    except (OSError, ValueError):
        return None
    return got if isinstance(got, dict) else None


class PolicyRunner:
    """One run. Use roborun_sdk.policy.run / start, or `roborun policy run`."""

    def __init__(self, provider, *, prompt, arms=None, seconds=60.0, model=None, options=None, consent=None, yes=False,
                 ask=None, shadow=False, dataset=True, max_tool_speed=None, wait_ready=WAIT_READY_S,
                 max_chunk_age=MAX_CHUNK_AGE_S, stall_limit=STALL_LIMIT_S, station_url=None, runtime=None,
                 camera_socket=None, config=None, dataset_root=None, on_event=None, clock=time.monotonic,
                 sleep=None, start_motion_s=START_MOTION_S, provider_name=None):
        from .registry import create
        self.provider = create(provider, options) if isinstance(provider, str) else provider
        own = getattr(self.provider, 'name', None)              # the provider's own name, else how it was named here
        self.provider_name = provider_name or (own if isinstance(own, str) and own and own != 'provider' else
                                               provider if isinstance(provider, str) else type(provider).__name__)
        self.options = dict(options or {})
        self.prompt = str(prompt or '').strip()
        self.arms_wanted = arms
        self.seconds = float(seconds if seconds is not None else (SHADOW_SECONDS if shadow else 60.0))
        self.model = model
        self.consent, self.yes, self.ask = consent, yes, ask
        self.shadow, self.dataset = shadow, dataset
        self.max_tool_speed = max_tool_speed
        self.wait_ready = min(max(0.0, float(wait_ready)), WAIT_READY_MAX_S)
        self.max_chunk_age = min(float(max_chunk_age), MAX_CHUNK_AGE_S)
        self.stall_limit = float(stall_limit)
        self.start_motion_s = float(start_motion_s)
        robot_json = dict(config) if config is not None else _config()
        self.config = robot_json or {}
        from .. import robot
        self.station = Station(station_url or self.config.get('station') or f'http://127.0.0.1:{robot.STATION_PORT}')
        self.runtime = Path(runtime) if runtime else robot_runtime(self.config)
        # robot-wide, as for every task: this computer's installed robot.json and its runtime's config.json decide, whatever
        # config= or runtime= say (they only say where things are); --no-dataset can only turn it off
        installed = self.installed = _config()
        backend = runtime_config(robot_runtime(installed))
        self.robot_dataset = robot_wants_dataset(installed, backend)
        # whether this robot records its tasks (config.json video_record), as the runtime's recorder decides: the installed
        # runtime's setting; with no robot.json installed (a development rig), the run's own runtime's
        self.record_settings = recording_settings(backend if installed is not None else runtime_config(self.runtime))
        self.recording_off = None                 # why this run records nothing, when it does not
        self.recording_cleanup = []               # the older recordings deleted to make room for this run's
        self.recording_capped_said = None         # the recorder cap already reported in the task record
        # whose recorder rules prune the recordings: the installed runtime's own (as for every task), else the run's runtime
        self.recorder_runtime = robot_runtime(installed) if installed is not None else self.runtime
        export = (backend or {}).get('lerobot_export') if isinstance((backend or {}).get('lerobot_export'), dict) else {}
        # config.json's own dataset folder and robot name, as the runtime's export uses them (lerobot_export.dataset_root_for)
        self.export_root = export.get('root') if isinstance(export.get('root'), str) and export.get('root') else None
        self.export_robot = export.get('robot') if isinstance(export.get('robot'), str) and export.get('robot') else None
        self.camera_socket = camera_socket
        self.dataset_root = dataset_root if dataset_root is not None else self.export_root
        self.on_event = on_event
        self.clock = clock
        self.sleep = sleep or (lambda s: self._stopping.wait(max(0.0, s)))
        self.id = secrets.token_hex(16)
        self.paths = Paths(self.runtime, self.id)
        self._stopping = threading.Event()        # stop() from the program (Python handle) or Ctrl-C
        self._stop_why = None
        self.hold_gen = None                      # the station's Stop counter when this run started (admit)
        self._hold_read_at = None
        self.lease = None
        self.lease_state = None                   # the newest lease view from a heartbeat or push reply
        self.counts = dict(requests=0, chunks_pushed=0, chunks_accepted=0, discarded_late=0, refused={}, provider_errors=0,
                           stale_ticks=0)
        self.rtts = []
        self.record = None
        self.recorder = None
        self.seq = 0
        self.buffer_s, self.buffer_at = 0.0, clock()       # what the station had left at the last accepted push
        self.events = queue.Queue()
        self.notices_seen = set()

    # ---- events --------------------------------------------------------------------------------------------------------
    def say(self, event, **fields):
        item = dict(event=event, run=self.id, **fields)
        self.events.put(item)
        if self.on_event is not None:
            try:
                self.on_event(item)
            except Exception:
                pass

    def stop(self, why='user'):
        self._stop_why = why
        self._stopping.set()

    # ---- before the lease ------------------------------------------------------------------------------------------------
    def admit(self):
        """needs() and the match check: offline, nothing sent, nothing leased. Returns (spec, needs, drive sides)."""
        if not self.shadow and not self.prompt:
            raise PolicyRefused('a policy run needs a prompt (what the arms should do)')
        if not 0.5 <= self.seconds <= MAX_SECONDS:
            raise PolicyRefused(f'--seconds must be between 0.5 and {MAX_SECONDS:g} (one lease is at most 600 s)')
        try:
            observed = self.station.observe()
        except StationRefused as exc:
            raise PolicyRefused(f'the arms\' station refused: {exc}') from None
        except OSError:
            raise PolicyRefused('the arms\' station is not running on this computer (roborun up starts it; policy runs '
                                'need an arm profile)') from None
        self.hold_gen = hold_generation(observed)
        arms = arms_of(observed, self.config)
        if not arms:
            raise PolicyRefused('the station has no arm a policy can drive')
        have = [a.side for a in arms]
        wanted = self.arms_wanted
        if wanted in (None, '', 'both', 'all'):
            drive = tuple(have) if wanted != 'both' else ('left', 'right')
        else:
            drive = tuple(wanted) if isinstance(wanted, (list, tuple)) else (str(wanted),)
        for side in drive:
            if side not in ('left', 'right'):
                raise PolicyRefused(f'--arms takes left, right or both, not {side!r}')
            if side not in have:
                raise PolicyRefused(f'this robot has no {side} arm' + (' (it is parked)' if side in ('left', 'right') else ''))
        cams = Cameras(self.camera_socket)
        roles = tuple(cams.roles())
        self.provider_log = self.paths.provider
        spec = RunSpec(prompt=self.prompt, arms=tuple(arms), cameras=roles, model=self.model, options=dict(self.options),
                       robot=str(self.config.get('robot') or ''), log_dir=self.paths.provider)
        try:
            needs = self.provider.needs(spec)
        except PolicyRefused:
            raise
        except Exception as exc:
            raise PolicyRefused(f'{self.provider_name}: {type(exc).__name__}: {exc}') from exc
        if not isinstance(needs, Needs):
            raise PolicyRefused(f'{self.provider_name}.needs() must return a Needs')
        missing = [s for s in needs.arms if s not in have]
        if missing:
            raise PolicyRefused(f'the {self.provider_name} provider drives the {missing[0]} arm, which this robot does not have')
        leased = tuple(s for s in ('left', 'right') if s in needs.arms and s in drive)
        if not leased:
            raise PolicyRefused(f'the {self.provider_name} provider drives {", ".join(needs.arms) or "no arm"}; '
                                f'--arms {"/".join(drive)} leaves nothing for it to move')
        for role in needs.cameras.values():
            if role not in roles:
                raise PolicyRefused(f'the {self.provider_name} provider needs the {role} camera, which is not configured or '
                                    'not delivering frames')
        if needs.cameras:
            try:
                import cv2  # noqa: F401
                import numpy  # noqa: F401
            except ImportError:
                raise PolicyRefused("camera frames for a policy need pip install 'roborun-sdk[cameras]'") from None
        self.face_blur = self._face_blur()
        if needs.remote and needs.cameras and self.face_blur:
            why = _blur_problem()
            if why:
                raise PolicyRefused(why)
        self.spec, self.needs, self.drive, self.leased, self.cams, self.roles = spec, needs, drive, leased, cams, roles
        self.arm_info = {a.side: a for a in arms}
        return spec, needs, leased

    def _face_blur(self):
        try:
            from ..robot import face_blur
            return face_blur(self.config) or face_blur(self.installed)   # the installed robot's setting is never loosened
        except BaseException:
            return True

    def check(self):
        spec, needs, leased = self.admit()
        key = _key_note(self.provider)
        return dict(ok=True, provider=self.provider_name, model=self.model, version=needs.version, arms=list(leased),
                    cameras=dict(needs.cameras), remote=needs.remote, sends=list(needs.sends), frame_hz=needs.frame_hz,
                    arm_models={a.side: a.model for a in spec.arms}, key=key)

    # ---- the run ---------------------------------------------------------------------------------------------------------
    def run(self):
        """Blocks until the run ends; returns a RunResult (or a ShadowReport for a shadow run). Raises PolicyRefused before
        anything was sent, PolicyNotStarted when the model was ready only after the start-motion limit."""
        self.t_start = self.clock()
        try:
            from .record import close_orphans
            close_orphans(self.runtime)                 # earlier runs whose process was killed: closed, honestly
        except Exception:
            pass
        self.admit()
        given = consent_mod.decide(self.provider_name, self.model, self.needs, self.face_blur, consent=self.consent,
                                   yes=self.yes, ask=self.ask, words=_consent_words(self.provider, self.needs, self.face_blur),
                                   say=lambda text: self.say('policy_consent_text', text=text),
                                   title=getattr(self.provider, 'title', None))
        self.consent_given = given
        if self.needs.remote:
            self.say('policy_consent', how=given['how'])
        if self.needs.remote and self.needs.cameras and self.face_blur:
            import os
            os.environ['ROBORUN_FACE_BLUR'] = '1'         # this process blurs what it sends, whatever it inherited
        self.paths.provider.mkdir(parents=True, exist_ok=True)
        self.observer = Observer(self.station, list(self.spec.arms), self.prompt, cameras=self.cams,
                                 roles=tuple(dict.fromkeys(self.needs.cameras.values())), frame_size=self.needs.frame_size,
                                 blur=bool(self.needs.remote and self.face_blur), clock=self.clock)
        self.worker = _Worker(self.provider)
        try:
            self.provider.start(self.spec)
        except PolicyRefused:
            raise
        except Exception as exc:
            raise PolicyRefused(f'{self.provider_name} could not start: {type(exc).__name__}: {exc}') from exc
        finally:
            self._provider_notices()
        self.say('policy_start', provider=self.provider_name, model=self.model, arms=list(self.leased),
                 shadow=self.shadow, version=self.needs.version)
        if self.shadow:
            try:
                return self._shadow()
            finally:
                self._finish_provider()
        self.record = TaskRecord(self.paths, prompt=self.prompt, provider=self.provider_name, model=self.model, policy=dict(
            provider=self.provider_name, provider_version=self.needs.version, model=self.model, arms=list(self.leased),
            remote=self.needs.remote, sends=list(self.needs.sends), consent_at=given.get('given_at'),
            face_blur=self.face_blur if self.needs.cameras else None))
        self.record.update(force=True, phase='Waiting for the model', arm=self.leased[0] if len(self.leased) == 1 else 'both')
        status, reason, message = 'failed', 'runner_error', 'the policy run failed'
        try:
            first = self._warm_up()
            self._lease(first)
            self._loop()
            status, reason, message = 'done', 'finished', 'Policy run finished'
        except _End as end:
            status, reason, message = end.status, end.stop_reason, end.message
        except PolicyNotStarted as exc:
            status, reason, message = 'not_started', 'model_ready_late', str(exc)
        except KeyboardInterrupt:
            status, reason, message = 'stopped', 'user', 'Stopped (Ctrl-C): the arms hold where they are'
        except Exception as exc:
            status, reason, message = 'failed', 'runner_error', f'{type(exc).__name__}: {exc}'[:300]
            self.say('policy_notice', text=traceback.format_exc()[-800:])
        finally:
            result = self._close(status, reason, message)
        if status == 'not_started':
            raise PolicyNotStarted(message)
        return result

    # ---- warm-up and the first chunk
    def _warm_up(self):
        """Ask for chunks until the first one arrives (nothing leased). Returns (obs, chunk, t_ready)."""
        deadline = self.t_start + self.wait_ready
        asked_again = False
        retry_at = None
        while True:
            self._check_user_stop()
            self._check_hold()
            now = self.clock()
            if now > deadline and not asked_again:
                raise _End('failed', 'provider_not_ready', f'{self.provider_name} was not ready within '
                                                            f'{self.wait_ready:.0f} s; nothing moved')
            if retry_at is not None and now < retry_at:
                self.sleep(min(0.25, retry_at - now))
                continue
            got = self._request_blocking()
            if got is None:
                self.sleep(0.05)
                continue
            obs, outcome, rtt = got
            if isinstance(outcome, ProviderError):
                if outcome.session_restarted:
                    raise _End('stopped', 'provider_session_restarted', 'the provider restarted its session; run again')
                if outcome.warming or outcome.retryable:
                    wait = outcome.retry_after_s if outcome.retry_after_s is not None else (
                        WARMING_RETRY_S if outcome.warming else random.uniform(0.25, 2.0))
                    self.say('policy_warming', text=f'model warming; {self.provider_name} says retry in {wait:.0f} s'
                             if outcome.warming else f'{self.provider_name}: {outcome}', retry_after_s=wait)
                    self.record.update(phase='The model is warming up')
                    retry_at = self.clock() + min(wait, max(0.0, deadline - self.clock()) + 0.01)
                    continue
                raise _End('failed', 'provider_error', f'{self.provider_name}: {outcome}')
            if isinstance(outcome, Exception):
                raise _End('failed', 'provider_error', f'{self.provider_name}: {type(outcome).__name__}: {outcome}')
            chunk = self._usable(outcome)
            if chunk is None:
                continue
            ready = self.clock()
            if not asked_again and ready - self.t_start > self.start_motion_s:
                self.say('policy_ready', late_s=round(ready - self.t_start, 1))
                if self.ask is None:
                    raise PolicyNotStarted('model ready; run again')
                answer = self.ask('The model is ready. Start moving the arms now? [y/N] ')
                if str(answer or '').strip().lower() not in ('y', 'yes'):
                    raise PolicyNotStarted('model ready; run again')
                asked_again = True                # a fresh observation and a fresh chunk: this one is stale
                continue
            if ready - obs.t > self.max_chunk_age:
                self.counts['discarded_late'] += 1
                continue
            self.say('policy_ready', after_s=round(ready - self.t_start, 1), rows=len(next(iter(chunk.rows.values()))))
            return obs, chunk

    def _request_blocking(self):
        """One refill tick and act() on the worker, waited for (warm-up and shadow). None when the tick had no observation."""
        obs = self._refill_tick()
        self._check_hold(self.observer.last_pose)
        if obs is None:
            return None
        self.worker.submit(obs, self.clock)
        while True:
            self._check_user_stop()
            self._check_hold()
            done = self.worker.result(timeout=0.1)
            if done is not None:
                obs2, outcome, rtt = done
                self._log_request(obs2, outcome, rtt)
                return done

    def _refill_tick(self):
        try:
            obs, why = self.observer.tick()
        except OSError:
            raise _End('failed', 'station_lost', 'the arms\' station stopped answering') from None
        if obs is None:
            self.counts['stale_ticks'] += 1
            if why == 'blur_unavailable' and self.observer.blur_failed_since is not None \
                    and self.clock() - self.observer.blur_failed_since > 3.0:
                raise _End('failed', 'blur_unavailable', 'face blur unavailable; nothing was sent')
            if why and why.startswith('camera ') and 'failed' in why:
                raise _End('failed', 'camera_stopped', f'{why.split(":")[0]} stopped')
            return None
        try:
            self.provider.observe(obs)
        except Exception as exc:
            self.say('policy_notice', text=f'{self.provider_name}.observe: {type(exc).__name__}: {exc}'[:300])
        return obs

    def _usable(self, chunk):
        """The chunk cut to the leased sides and checked for shape, else None (counted)."""
        if not isinstance(chunk, Chunk) or not isinstance(chunk.rows, dict):
            return self._bad_shape('act() must return a Chunk')
        rows = {}
        for side in self.leased:
            table = chunk.rows.get(side)
            if table is None:
                return self._bad_shape(f'no rows for the {side} arm')
            info = self.arm_info[side]
            width = info.joints + (1 if info.gripper else 0)
            try:
                table = [[float(v) for v in r] for r in table]
            except (TypeError, ValueError):
                return self._bad_shape(f'the {side} rows are not numbers')
            if not table or any(len(r) != width for r in table):
                shape = (len(table), len(table[0]) if table else 0)
                return self._bad_shape(f'{side} rows have shape {shape}; need (T, {width})')
            if not all(math.isfinite(v) for r in table for v in r):
                return self._bad_shape(f'a {side} row is not finite')
            rows[side] = table
        try:
            hz = float(chunk.hz)
        except (TypeError, ValueError):
            return self._bad_shape('hz must be a number')
        self._shape_fails = 0
        self._last_shape_why = None
        return Chunk(rows, hz=hz, meta=dict(chunk.meta or {}))

    def _bad_shape(self, why):
        same = why == getattr(self, '_last_shape_why', None)
        self._shape_fails = (getattr(self, '_shape_fails', 0) + 1) if same else 1
        self._last_shape_why = why
        self.counts['refused']['shape'] = self.counts['refused'].get('shape', 0) + 1
        self.say('policy_refused', by='runner', why=why)
        if self._shape_fails >= SAME_SHAPE_FAILS:
            raise _End('failed', 'provider_bad_chunks', f'{self.provider_name} sent the same malformed chunk '
                                                         f'{SAME_SHAPE_FAILS} times: {why}')
        return None

    # ---- the lease and the loop
    def _lease(self, first):
        obs, chunk = first
        gripper = all(self.arm_info[s].gripper for s in self.leased)
        cap = self.max_tool_speed
        self._check_hold(force=True)
        try:
            reply = self.station.lease(self.leased, min(600.0, self.seconds + LEASE_MARGIN_S), self.id, gripper=gripper,
                                       max_tool_speed_m_s=cap, since_hold_generation=self.hold_gen)
        except StationRefused as exc:
            self._check_hold(force=True)                 # a Stop between the last read and the lease: the station said so
            raise _End('failed', 'lease_refused', f'the station refused the run: {exc}') from None
        self.lease = reply['lease']
        self.lease_state = reply['lease']
        self.limits = reply.get('limits') or {}
        self.t_lease = self.clock()
        self.record.update(force=True, phase='Running the policy', policy=dict(
            self.record.record['policy'], hz=chunk.hz, limits=self.limits, tool_speed_m_s=self.lease.get('tool_speed_m_s'),
            tool_speed_active=self.lease.get('tool_speed_active')))
        if not self.lease.get('tool_speed_active'):
            self.say('policy_notice', text='the tool speed cap is not active on these arms (no arm model); the joint '
                                           'speed limit still applies')
        self.say('policy_lease', lease=self.lease['id'], arms=list(self.leased), tool_speed_m_s=self.lease.get('tool_speed_m_s'))
        self._hb_stop = threading.Event()
        self._hb = threading.Thread(target=self._heartbeat, name='policy-heartbeat', daemon=True)
        self._hb.start()
        self._start_recording()
        self.last_accept = self.clock()
        self._push(obs, chunk)

    def _dataset_robot(self):
        """The dataset's robot name, as the runtime's export names it: config.json lerobot_export.robot, else the installed
        robot.json's robot (never a caller's own config=, which cannot put an episode into another robot's dataset)."""
        for name in (self.export_robot, (self.installed or {}).get('robot'), self.config.get('robot')):
            if isinstance(name, str) and name:
                return name
        return 'robot'

    def _start_recording(self):
        """The recording, when this robot records its tasks (config.json video_record: enabled, its cameras, its caps): every
        arm the station has and the run's cameras that the setting lists. Otherwise nothing, and the reason is kept."""
        cfg = self.record_settings
        room = None
        if not cfg['enabled']:
            why = cfg['why']
        else:
            # the same retention as an ordinary task's recording: the oldest recordings go until this run fits the caps
            try:
                self.paths.recording.mkdir(parents=True, exist_ok=True)
                room = make_room(cfg, self.paths.recording, self.paths.clip.parent, runtime=self.recorder_runtime)
                why = room['why']
            except OSError as exc:
                why = f'its recording folder could not be made ({type(exc).__name__}: {exc})'[:200]
            self.recording_cleanup = list((room or {}).get('cleanup') or [])
            if self.recording_cleanup:
                freed = sum(c.get('bytes') or 0 for c in self.recording_cleanup) / MB
                self.say('policy_notice', text=f'deleted the {len(self.recording_cleanup)} oldest recording(s) '
                                               f'({freed:.0f} MB) to make room, as for every task (config.json '
                                               f'video_record.total_cap_mb)')
        if not why:
            sides = [a.side for a in self.spec.arms]
            roles = [r for r in self.roles if r in cfg['cameras']]
            day_room = cfg['day_cap_mb'] * MB - (room or {}).get('day_bytes', 0)
            try:
                self.recorder = Recorder(self.paths, Station(self.station.url), self.cams, roles, prompt=self.prompt,
                                         sides=sides, jaw_range={a.side: a.jaw_range for a in self.spec.arms},
                                         cap_bytes=cfg['run_cap_mb'] * MB, day_room_bytes=day_room,
                                         min_free_bytes=cfg['min_free_mb'] * MB, cleanup=self.recording_cleanup).start()
                return
            except OSError as exc:                     # a disk that filled between the check and the first write
                self.recorder = None
                why = f'its recording could not be written ({type(exc).__name__}: {exc})'[:200]
        self.recording_off = why
        for d in (self.paths.recording, self.paths.clip):   # nothing recorded: no empty folders left behind
            try:
                for f in d.iterdir() if d == self.paths.recording else ():
                    if f.is_file() and not f.is_symlink():
                        f.unlink()
                d.rmdir()
            except OSError:
                pass
        self.say('policy_notice', text=f'this run is not recorded: {why}')
        self._note_recording()

    def _recording_view(self, rec_info=None):
        """The task record's `recording`: recorded false and why; or the frames, and the cap that ended them (in words);
        and the older recordings deleted to make room."""
        out = {}
        if self.recording_off:
            out = dict(recorded=False, why=self.recording_off)
        elif self.recorder is not None or rec_info:
            capped = (rec_info or {}).get('capped') or (self.recorder.capped if self.recorder is not None else None)
            out = dict(recorded=True, frames=(rec_info or {}).get('frames'))
            if capped:
                out.update(capped=capped, why=f'recording stopped early: {cap_text(capped, self.record_settings)}')
        else:
            return None
        if self.recording_cleanup:
            out['cleanup'] = [dict(dir=c.get('dir'), created_at=c.get('created_at'), bytes=c.get('bytes'))
                              for c in self.recording_cleanup]
        return out

    def _note_recording(self):
        """Puts the recording's state into the task record when it changes: a run never stops recording silently."""
        if self.record is None:
            return
        capped = self.recorder.capped if self.recorder is not None else None
        if self.recorder is not None and capped == self.recording_capped_said:
            return
        if capped:
            self.say('policy_notice', text=f'this run\'s recording stopped: {cap_text(capped, self.record_settings)}; '
                                           f'the run goes on')
        self.recording_capped_said = capped
        self.record.update(force=True, recording=self._recording_view())

    def _heartbeat(self):
        period = 1.0 / HEARTBEAT_HZ
        while not self._hb_stop.wait(period):
            try:
                reply = self.station.heartbeat(self.lease['id'])
                self.lease_state = reply.get('lease') or self.lease_state
            except StationRefused as exc:
                self.lease_state = dict(self.lease_state or {}, active=False, reason='lease_lost', error=str(exc))
            except OSError:
                pass                                   # the station's own heartbeat watchdog decides; the loop sees it

    def _lease_over(self):
        lease = self.lease_state or {}
        if lease.get('active', True):
            return None
        reason = lease.get('reason') or 'unknown'
        status = REASONS.get(reason, 'failed')
        why = lease.get('stop_reason')
        if reason == 'supervisor_stop':
            return _End('stopped', 'operator_stop', 'Stopped: the arms hold where they are')
        if status == 'done':
            return _End('done', reason, 'Policy run finished')
        text = {'client_disconnected': 'the station lost this run\'s heartbeat; the arms held',
                'policy_error': f'the station ended the run: {why or reason}',
                'station_error': 'an arm stopped answering; the arms held'}.get(reason, f'the station ended the run ({reason})')
        return _End('failed', reason, text)

    def _push(self, obs, chunk):
        now = self.clock()
        skip = max(0.0, now - obs.t)
        if skip > self.max_chunk_age:
            self.counts['discarded_late'] += 1
            self.say('policy_refused', by='runner', why=f'chunk {skip:.2f} s old; discarded')
            return False
        self.seq += 1
        rows = {side: chunk.rows[side] for side in self.leased}
        if not self.lease.get('gripper', True):         # a lease without gripper columns: the jaws keep their targets
            rows = {side: [r[:self.arm_info[side].joints] for r in table] for side, table in rows.items()}
        body = dict(lease_id=self.lease['id'], seq=self.seq, hz=chunk.hz, skip_s=round(skip, 4), rows=rows)
        self.counts['chunks_pushed'] += 1
        try:
            ok, reply = self.station.chunk(body)
        except StationRefused as exc:
            self.lease_state = dict(self.lease_state or {}, active=False, reason='lease_lost', error=str(exc))
            return False
        except OSError:
            raise _End('failed', 'station_lost', 'the arms\' station stopped answering') from None
        if reply.get('lease'):
            self.lease_state = reply['lease']
        self._chunk_log(obs, chunk, skip, ok, reply)
        if ok:
            self.counts['chunks_accepted'] += 1
            self.last_accept = self.clock()
            self.buffer_s = reply.get('buffer_s', 0.0)
            self.buffer_at = self.clock()
            self.say('policy_chunk', seq=self.seq, rows=len(next(iter(body['rows'].values()))), skip_s=body['skip_s'],
                     skipped_rows=reply.get('skipped_rows'), buffer_s=reply.get('buffer_s'))
        else:
            code = reply.get('code') or 'refused'
            if code != 'lease':
                self.counts['refused'][code] = self.counts['refused'].get(code, 0) + 1
                self.say('policy_refused', by='station', code=code, why=reply.get('refused'))
        return ok

    def _loop(self):
        period = 1.0 / LOOP_HZ
        end_at = self.t_lease + self.seconds
        next_observe = self.clock()
        observe_period = 1.0 / max(0.1, float(self.needs.frame_hz or 1.0))
        retry_at = None
        hold_said = False
        while True:
            t = self.clock()
            over = self._lease_over()
            if over is not None:
                raise over
            if self._stopping.is_set():
                why = self._stop_why
                self._end_lease(why if str(why).startswith('SIG') else 'stopped by the program' if why != 'user' else 'Ctrl-C')
                if str(why).startswith('SIG'):
                    raise _End('stopped', 'signal', f'Stopped ({why}): the arms hold where they are')
                raise _End('stopped', 'user', 'Stopped: the arms hold where they are')
            if t >= end_at:
                self._end_lease('seconds elapsed')
                raise _End('done', 'finished', f'Policy run finished: {self.counts["chunks_accepted"]} chunks played')
            remaining = max(0.0, self.buffer_s - (t - self.buffer_at))
            done = self.worker.result(timeout=0)
            if done is not None:
                obs, outcome, rtt = done
                self._log_request(obs, outcome, rtt)
                if isinstance(outcome, ProviderError):
                    self.counts['provider_errors'] += 1
                    if outcome.session_restarted:
                        self._end_lease('provider session restarted')
                        raise _End('stopped', 'provider_session_restarted', 'the provider restarted its session; run again')
                    if outcome.retryable or outcome.warming:
                        wait = outcome.retry_after_s if outcome.retry_after_s is not None else random.uniform(0.25, 2.0)
                        retry_at = self.clock() + wait
                        self.say('policy_hold', why=f'{self.provider_name}: {outcome}', retry_after_s=round(wait, 2))
                    else:
                        self._end_lease('provider error')
                        raise _End('failed', 'provider_error', f'{self.provider_name}: {outcome}')
                elif isinstance(outcome, Exception):
                    self._end_lease('provider error')
                    raise _End('failed', 'provider_error', f'{self.provider_name}: {type(outcome).__name__}: {outcome}')
                else:
                    chunk = self._usable(outcome)
                    if chunk is not None:
                        if self._push(obs, chunk):
                            hold_said = False
                    continue
            if self.clock() - self.last_accept > self.stall_limit:
                self._end_lease('provider stalled')
                raise _End('failed', 'provider_stalled', f'no chunk from {self.provider_name} for {self.stall_limit:g} s; '
                                                         'the arms held')
            if remaining <= 0 and not hold_said and self.counts['chunks_accepted']:
                self.say('policy_hold', why='the buffer ran dry: the arms hold until the next chunk')
                hold_said = True
            if not self.worker.busy() and (retry_at is None or t >= retry_at) and self._refill_due(remaining):
                obs = self._refill_tick()
                if obs is not None:
                    retry_at = None
                    self.worker.submit(obs, self.clock)
                    next_observe = self.clock() + observe_period
            elif t >= next_observe:
                self._refill_tick()                    # an observe tick: the provider keeps its own history
                next_observe = t + observe_period
            self._update_record()
            self.sleep(max(0.0, period - (self.clock() - t)))

    def _refill_due(self, remaining):
        fn = getattr(self.provider, 'refill_due', None)
        if callable(fn):
            try:
                executed = max(0, int((self.clock() - self.buffer_at) * 30))
                return bool(fn(executed, max(0, int(remaining * 30))))
            except Exception:
                pass
        lead = (min(self.rtts[-5:]) / 1000.0) if self.rtts else 1.0
        return remaining <= lead + 0.25

    def _end_lease(self, why):
        if self.lease is None or not (self.lease_state or {}).get('active', True):
            return
        try:
            reply = self.station.end(self.lease['id'], why)
            self.lease_state = reply.get('lease') or self.lease_state
        except (StationRefused, OSError):
            pass

    def _check_user_stop(self):
        if self._stopping.is_set():
            raise _End('stopped', 'signal' if str(self._stop_why).startswith('SIG') else 'user', 'Stopped before anything moved')

    def _check_hold(self, pose=None, force=False):
        """Before the lease: a Stop pressed anywhere since admit() (roborun stop, the page or viewer Stop, any hold) ends the
        run with nothing leased. The station has no lease to end yet, so the runner reads its Stop counter, and the runs it
        was asked to stop by id (this task's own Stop, POST /supervisor/policy/stop)."""
        if pose is None:
            now = self.clock()
            if not force and self._hold_read_at is not None and now - self._hold_read_at < HOLD_POLL_S:
                return
            self._hold_read_at = now
            try:
                pose = self.station.pose()
            except (StationRefused, OSError):
                return                                 # the next refill tick reports a station that stopped answering
        if not isinstance(pose, dict):
            return
        if self.id in (pose.get('stopped_runs') or (pose.get('status') or {}).get('stopped_runs') or ()):
            raise _End('stopped', 'operator_stop', 'Stopped before anything moved')
        got = hold_generation(pose)
        if self.hold_gen is None or got is None or got == self.hold_gen:
            return
        if got > self.hold_gen:
            raise _End('stopped', 'operator_stop', 'Stopped before anything moved')
        raise _End('failed', 'station_restarted', 'the arms\' station restarted before anything moved; run again')

    def _update_record(self):
        if self.record is None:
            return
        self.record.update(safety=self._safety())
        if self.recorder is not None and self.recorder.capped != self.recording_capped_said:
            self._note_recording()

    def _safety(self):
        pose_policy = {}
        try:
            pose_policy = ((self.observer.last_pose or {}).get('execution_lease') or {}).get('policy') or {}
        except Exception:
            pass
        lease = self.lease_state or {}
        stats = lease.get('policy') or pose_policy
        return dict(chunks_played=stats.get('chunks', self.counts['chunks_accepted']),
                    refused=dict(stats.get('refused') or {}, **{f'runner_{k}': v for k, v in self.counts['refused'].items()
                                                                if k == 'shape'}),
                    discarded_late=self.counts['discarded_late'], dry_ticks=stats.get('dry_ticks', 0),
                    clipped_ticks=stats.get('clipped_ticks', 0), stale_ticks=self.counts['stale_ticks'])

    # ---- logs
    def _provider_notices(self):
        """The provider's own notices (an optional drain_notices() -> [str]), each shown once as a policy_notice."""
        fn = getattr(self.provider, 'drain_notices', None)
        if not callable(fn):
            return
        try:
            notes = list(fn() or ())
        except Exception:
            return
        for text in notes:
            text = str(text)[:500]
            if text and text not in self.notices_seen:
                self.notices_seen.add(text)
                self.say('policy_notice', text=text)

    def _log_request(self, obs, outcome, rtt):
        self._provider_notices()
        self.counts['requests'] += 1
        ok = isinstance(outcome, Chunk)
        if ok:
            self.rtts.append(rtt * 1000.0)
        line = dict(t=round(time.time(), 3), rtt_ms=round(rtt * 1000.0, 1), outcome='chunk' if ok else 'error',
                    chunk_age_s=round(self.clock() - obs.t, 4) if obs is not None else None)
        if ok:
            meta = dict(outcome.meta or {})
            line.update(rows=len(next(iter(outcome.rows.values()), [])), hz=outcome.hz,
                        provider_ms=meta.get('provider_ms'), request_bytes=meta.get('request_bytes'),
                        response_bytes=meta.get('response_bytes'))
        elif isinstance(outcome, ProviderError):
            line.update(code=outcome.code, retryable=outcome.retryable, warming=outcome.warming,
                        retry_after_s=outcome.retry_after_s, error=str(outcome)[:300])
        else:
            line.update(error=f'{type(outcome).__name__}: {outcome}'[:300])
        try:
            self.paths.policy.mkdir(parents=True, exist_ok=True)
            append_line(self.paths.policy / 'requests.jsonl', line)
        except OSError:
            pass

    def _chunk_log(self, obs, chunk, skip, ok, reply):
        try:
            append_line(self.paths.policy / 'chunks.jsonl', dict(
                seq=self.seq, observed_at=obs.t, pushed_at=self.clock(), skip_s=round(skip, 4), hz=chunk.hz, rows=chunk.rows,
                meta=_plain(chunk.meta), accepted=ok, skipped_rows=reply.get('skipped_rows'),
                buffer_s_before=None, refused=reply.get('refused'), code=reply.get('code')))
        except (OSError, ValueError, TypeError):
            pass

    # ---- the end
    def _finish_provider(self):
        def end():
            for fn in ('end_episode', 'close'):
                try:
                    getattr(self.provider, fn, lambda: None)()
                except Exception:
                    pass
        t = threading.Thread(target=end, name='policy-provider-end', daemon=True)
        t.start()
        t.join(END_CAP_S)
        self.worker.close()

    def _close(self, status, reason, message):
        if str(reason or '').startswith('provider_') or reason in ('model_ready_late', 'runner_error'):
            message = provider_text(message)            # a provider's words never read as Roborun's own errors
        if getattr(self, '_hb_stop', None) is not None:
            self._hb_stop.set()
        if self.lease is not None and status != 'not_started':
            self._end_lease(reason)
            try:
                reply = self.station.heartbeat(self.lease['id'])
                self.lease_state = reply.get('lease') or self.lease_state
            except Exception:
                pass
        self._finish_provider()
        self.say('policy_end', status=status, stop_reason=reason, message=message)
        rec_info, episode = None, None
        if self.recorder is not None:
            rec_info = self.recorder.stop(reason)
            if rec_info.get('capped'):
                rec_info['why'] = f"recording stopped early: {cap_text(rec_info['capped'], self.record_settings)}"
            wants, why_not = self.robot_dataset
            if not wants:
                episode = dict(recorded=False, why=why_not)
            elif self.dataset:
                profile = str(self.config.get('profile') or 'custom')
                episode = write_episode(self.recorder, robot=self._dataset_robot(), profile=profile,
                                        prompt=self.prompt, dataset_root=self.dataset_root,
                                        config=dict(self.config, sim=self.config.get('sim') or (self.installed or {}).get('sim')),
                                        extras=dict(task_id=self.id, provider=self.provider_name, model=self.model,
                                                    outcome=status if status in ('done', 'stopped', 'failed') else 'failed',
                                                    stop_reason=reason, mode='run', source='roborun policy run',
                                                    gripper_units='station',
                                                    jaw_range={a.side: list(a.jaw_range or ()) for a in self.spec.arms}))
            else:
                episode = dict(recorded=False, why='--no-dataset')
        elif self.recording_off:                       # the robot does not record its tasks: no recording, no episode
            rec_info = dict(recorded=False, why=self.recording_off)
            if self.recording_cleanup:
                rec_info['cleanup'] = list(self.recording_cleanup)
            episode = dict(recorded=False, why='no recording')
        summary = self._summary(status, reason, message, rec_info, episode)
        try:
            write_json(self.paths.policy / 'summary.json', summary)
        except OSError:
            pass
        if self.record is not None:
            final_status = status if status in ('done', 'stopped', 'failed') else 'failed'
            self.record.close(final_status, final=message, error=message if final_status == 'failed' else None,
                              stop_reason=reason, safety=self._safety(),
                              recording=self._recording_view(rec_info),
                              dataset=dict(recorded=(episode or {}).get('recorded'), why=(episode or {}).get('why')))
        return RunResult(task_id=self.id, status=status, stop_reason=reason, message=message, summary=summary)

    def _summary(self, status, reason, message, rec_info, episode):
        return dict(id=self.id, provider=self.provider_name, model=self.model, status=status, stop_reason=reason,
                    message=message, requests=self.counts['requests'], chunks_pushed=self.counts['chunks_pushed'],
                    chunks_accepted=self.counts['chunks_accepted'], discarded_late=self.counts['discarded_late'],
                    refused=dict(self.counts['refused']), provider_errors=self.counts['provider_errors'],
                    rtt_ms=dict(p50=_percentile(self.rtts, 50), p90=_percentile(self.rtts, 90),
                                max=round(max(self.rtts), 1) if self.rtts else None),
                    safety=self._safety() if self.lease is not None else None, recording=rec_info, dataset=episode,
                    arms=list(getattr(self, 'leased', ())), seconds=self.seconds)

    # ---- shadow: nothing leased, nothing moves
    def _shadow(self):
        from roborun_station import policy as pol
        report = dict(run=self.id, provider=self.provider_name, model=self.model, chunks=[], seconds=self.seconds)
        end_at = self.clock() + self.seconds
        buffer_end = self.clock()
        retry_at = None
        try:
            while self.clock() < end_at and not self._stopping.is_set():
                now = self.clock()
                lead = (min(self.rtts[-5:]) / 1000.0) if self.rtts else 1.0
                if (retry_at is not None and now < retry_at) or buffer_end - now > lead + 0.25:
                    self.sleep(0.02)
                    continue
                try:
                    got = self._request_blocking()
                except _End:
                    break
                if got is None:
                    self.sleep(0.05)
                    continue
                obs, outcome, rtt = got
                if isinstance(outcome, ProviderError):
                    if outcome.retryable or outcome.warming:
                        retry_at = self.clock() + (outcome.retry_after_s or (WARMING_RETRY_S if outcome.warming else 1.0))
                        continue
                    report['error'] = str(outcome)
                    break
                if isinstance(outcome, Exception):
                    report['error'] = f'{type(outcome).__name__}: {outcome}'
                    break
                chunk = self._usable(outcome)
                if chunk is None:
                    continue
                arrival = self.clock()
                skip = arrival - obs.t
                row = self._shadow_row(pol, obs, chunk, skip, arrival)
                report['chunks'].append(row)
                buffer_end = arrival + max(0.0, (len(next(iter(chunk.rows.values()))) - 1) / chunk.hz - skip)
                try:
                    append_line(self.paths.policy / 'chunks.jsonl', dict(observed_at=obs.t, arrival=arrival, skip_s=skip,
                                                                         hz=chunk.hz, rows=chunk.rows, meta=_plain(chunk.meta)))
                except (OSError, TypeError, ValueError):
                    pass
                self.say('policy_chunk', shadow=True, seam_rad=row['seam_rad'], would_refuse=row['would_refuse'])
        except _End as end:
            report['error'] = end.message
        rows = report['chunks']
        mx = lambda k: round(max((r[k] for r in rows if r[k] is not None), default=0.0), 4) if rows else None
        refusals = {}
        for r in rows:
            if r['would_refuse']:
                refusals[r['would_refuse']] = refusals.get(r['would_refuse'], 0) + 1
        out = ShadowReport(run_id=self.id, chunks=len(rows), seam_rad_max=mx('seam_rad'),
                           step_from_measured_rad_max=mx('step_from_measured_rad'), joint_speed_rad_s_max=mx('joint_speed_rad_s'),
                           would_refuse=refusals, path=str(self.paths.policy / 'shadow.json'))
        report.update(summary=dict(out.__dict__), rtt_ms=dict(p50=_percentile(self.rtts, 50), p90=_percentile(self.rtts, 90)))
        write_json(self.paths.policy / 'shadow.json', report)
        self.say('policy_end', status='shadow', chunks=len(rows), seam_rad_max=out.seam_rad_max,
                 joint_speed_rad_s_max=out.joint_speed_rad_s_max, would_refuse=refusals)
        return out

    def _shadow_row(self, pol, obs, chunk, skip, arrival):
        """As the station would see this chunk: the seam, the step from the measured pose, the fastest row, a refusal."""
        sides = list(self.leased)
        info = self.arm_info
        commanded = {}
        for s in sides:
            st = obs.arms[s]
            jaw = None
            if info[s].gripper and st.commanded_gripper is not None:
                jaw = pol.jaw_from_fraction(st.commanded_gripper, info[s].jaw_range)
            commanded[s] = (list(st.commanded_rad), jaw)
        body = dict(hz=chunk.hz, skip_s=skip, rows={s: chunk.rows[s] for s in sides})
        would, first = None, 0
        try:
            _, first = pol.check_chunk(body, sides=sides, joints={s: info[s].joints for s in sides},
                                       grippers={s: info[s].gripper for s in sides},
                                       jaw_ranges={s: info[s].jaw_range for s in sides}, commanded=commanded, arrival=arrival)
        except pol.ChunkRefused as exc:
            would = exc.code
        seam = step = speed = None
        for s in sides:
            rows = chunk.rows[s]
            n = info[s].joints
            k = min(first, len(rows) - 1)
            seam_s = max(abs(a - b) for a, b in zip(rows[k][:n], commanded[s][0]))
            step_s = max(abs(a - b) for a, b in zip(rows[k][:n], obs.arms[s].q_rad))
            speed_s = max((abs(rows[i + 1][j] - rows[i][j]) * chunk.hz for i in range(len(rows) - 1) for j in range(n)),
                          default=0.0)
            seam = seam_s if seam is None else max(seam, seam_s)
            step = step_s if step is None else max(step, step_s)
            speed = speed_s if speed is None else max(speed, speed_s)
        return dict(skip_s=round(skip, 4), skipped_rows=first, seam_rad=round(seam, 5), step_from_measured_rad=round(step, 5),
                    joint_speed_rad_s=round(speed, 4), would_refuse=would)


class _Worker:
    """act() on one thread, one call in flight."""

    def __init__(self, provider):
        self.provider = provider
        self.jobs = queue.Queue()
        self.done = queue.Queue()
        self._busy = False
        self.thread = threading.Thread(target=self._run, name='policy-act', daemon=True)
        self.thread.start()

    def _run(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            obs, clock = job
            t0 = clock()
            try:
                outcome = self.provider.act(obs)
            except Exception as exc:                 # noqa: BLE001 - every failure is the provider's, reported as such
                outcome = exc
            self.done.put((obs, outcome, clock() - t0))

    def submit(self, obs, clock):
        self._busy = True
        self.jobs.put((obs, clock))

    def busy(self):
        return self._busy

    def result(self, timeout=0):
        try:
            got = self.done.get(timeout=timeout) if timeout else self.done.get_nowait()
        except queue.Empty:
            return None
        self._busy = False
        return got

    def close(self):
        self.jobs.put(None)


def _plain(value):
    try:
        json.dumps(value, default=str)
        return value
    except (TypeError, ValueError):
        return str(value)[:500]


def _blur_problem():
    """Why face blur cannot run here (a remote provider then gets nothing), or None."""
    try:
        import cv2
        import numpy  # noqa: F401
        if not hasattr(cv2, 'FaceDetectorYN'):
            raise ImportError('cv2.FaceDetectorYN')
        from .. import camera_privacy
        camera_privacy.verified_model()
    except Exception:
        return "face blur needs OpenCV with its face detector: pip install 'roborun-sdk[cameras]'"
    return None


def _consent_words(provider, needs, face_blur):
    fn = getattr(provider, 'consent_text', None)
    if callable(fn):
        try:
            return fn(needs, face_blur)
        except Exception:
            pass
    return None


def _key_note(provider):
    fn = getattr(provider, 'key_source', None)
    if callable(fn):
        try:
            return fn()
        except Exception:
            return None
    return None
