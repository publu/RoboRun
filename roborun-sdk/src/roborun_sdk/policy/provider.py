"""The provider interface of a policy run: what Roborun gives a policy and what it takes back.

New in this release; it may change within 0.1 (end-effector poses, named parts and IK are coming), so it is not on the stable
surface yet. Standard library only.

A provider never opens arms or cameras. Roborun hands it observations (measured joints, the target the station last played,
camera frames) and takes back chunks: per arm, rows of absolute targets [q_1..q_n, gripper] at `hz`, in the arm's own joint
order, radians, and the gripper as an open fraction from 0 (closed) to 1 (open). The station converts the fraction to its own
jaw units, checks every chunk, and plays it through every Roborun limit (the per-tick clip, the joint and tool speed limits,
your rules file, Stop).

    class MyPolicy(Provider):
        name = 'mine'
        def needs(self, spec):  return Needs(arms=('left',), cameras={}, frame_hz=5, frame_size=None, remote=False,
                                             sends=(), version='mine 1')
        def act(self, obs):     q = list(obs.arms['left'].commanded_rad); return Chunk({'left': [q + [1.0]] * 30}, hz=30)

`needs(spec)` must not touch the network: `roborun policy check` calls only it. `start(spec)` is called after consent (for a
remote provider) and may connect. `act(obs)` may take seconds: it runs on its own thread, never on the station's tick or
the Stop path.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence


class PolicyError(Exception):
    """Base of the errors a policy run raises to programs."""


class PolicyRefused(PolicyError):
    """The run cannot start (a mismatch, a missing extra, no consent); nothing was sent and nothing moved."""


class PolicyNotStarted(PolicyError):
    """The model became ready only after the start-motion limit: nothing moved; run again."""


@dataclass(frozen=True)
class ArmInfo:
    """One arm a run may drive, from the station's observe and the robot's profile."""
    side: str                          # "left" | "right"
    model: str                         # catalog id (data/arm_models/<id>), e.g. "yam"; "unknown" when the profile does not say
    semantics: str | None              # registry semantics id when the arm has one
    joints: int                        # arm joints, without the gripper
    limits_rad: tuple                  # per joint (lo, hi)
    gripper: bool                      # True: rows carry a gripper open fraction 0 (closed) .. 1 (open)
    home_rad: tuple                    # where the arm was when the run was set up (the station does not report a Home)
    jaw_range: tuple | None = None     # the station's jaw units (closed, open); providers never need them


@dataclass(frozen=True)
class RunSpec:
    prompt: str
    arms: tuple                        # ArmInfo of the arms this run may move, left first
    cameras: tuple                     # Roborun camera roles available: a subset of ("top", "left", "right")
    model: str | None
    options: Mapping[str, Any]
    robot: str                         # the robot's name, for the provider's own logs
    log_dir: Path                      # <runtime>/log/policy/<id>/provider/: the provider's own files (never uploaded)


@dataclass(frozen=True)
class Needs:
    """What a provider needs, from needs(): checked before consent, start() and any lease."""
    arms: tuple                        # the sides it will drive
    cameras: Mapping[str, str]         # provider role -> Roborun role, e.g. {"overhead": "top"}
    frame_hz: float                    # how often it wants observations (also how often frames are blurred)
    frame_size: tuple | None           # (w, h) to downscale to before JPEG; None = as captured
    remote: bool                       # True: frames and state leave this computer (consent; blur fails closed)
    sends: tuple                       # plain words for the consent text
    version: str                       # the provider and its package version, kept in the record


@dataclass(frozen=True)
class ArmState:
    q_rad: tuple                       # measured
    gripper: float | None              # measured, as an open fraction
    commanded_rad: tuple               # the target the station last played, from the same pose read
    commanded_gripper: float | None


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    width: int
    height: int
    captured_at: float                 # time.monotonic() of the exposure


@dataclass(frozen=True)
class Observation:
    t: float                           # time.monotonic() when the pose was read: a chunk from this observation is that old
    arms: Mapping[str, ArmState]
    frames: Mapping[str, Frame]        # Roborun role -> Frame (blurred when the provider is remote)
    prompt: str


@dataclass(frozen=True)
class Chunk:
    rows: Mapping[str, Sequence[Sequence[float]]]   # side -> T rows of [q_1..q_n, gripper], absolute, rad / 0..1
    hz: float                                       # row rate
    meta: Mapping[str, Any] = field(default_factory=dict)   # provider ids and timing; kept on this computer only


class ProviderError(Exception):
    """What act() raises when it has no chunk this time. The arm holds while the run decides."""

    def __init__(self, message='', *, retryable=False, retry_after_s=None, code=None, warming=False,
                 session_restarted=False):
        super().__init__(message)
        self.retryable = bool(retryable)
        self.retry_after_s = retry_after_s
        self.code = code
        self.warming = bool(warming)               # the model is not ready yet
        self.session_restarted = bool(session_restarted)   # the provider restarted its episode: the run ends


class Provider:
    """Subclass this (or write a class with the same methods). Defaults: observe, start, end_episode and close do nothing."""
    name = 'provider'

    def needs(self, spec: RunSpec) -> Needs:
        raise NotImplementedError

    def start(self, spec: RunSpec) -> None:
        pass

    def observe(self, obs: Observation) -> None:
        pass

    def act(self, obs: Observation) -> Chunk:
        raise NotImplementedError

    def end_episode(self) -> None:
        pass

    def close(self) -> None:
        pass


def plain_needs(arms, *, cameras=None, frame_hz=5.0, version='', remote=False, sends=()):
    """A Needs for a local provider that drives `arms` and wants these cameras ({role: role})."""
    return Needs(arms=tuple(arms), cameras=dict(cameras or {}), frame_hz=float(frame_hz), frame_size=None, remote=remote,
                 sends=tuple(sends), version=version)
