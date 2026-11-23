"""Are the wrist cameras on the right arms? Judged from what the robot does, without moving anything.

A wrist camera rides on its arm, so while one arm moves and the other is still, the moving arm's camera sees far more change
than the other one. This looks only at those moments. It answers `ok` or `swapped` when an arm has moved alone for long enough
and the difference is clear; otherwise `unknown`, never a guess. It says nothing about the top camera.

Checked on 127 YAM recordings, half used to choose the numbers below and half held back: 119 got an answer and all 119
were right, 8 came back unknown. Those recordings are from one robot, so another rig may answer `unknown` more often.
"""
import json
import time
import urllib.request

import numpy as np

SAMPLE_HZ = 3.          # the rate the numbers below were measured at: one sample every third of a second
MOVING_RAD = 0.004      # largest joint travel between two samples that counts as moving
STILL_RAD = 0.002       # and as staying still
MIN_ALONE = 8           # samples of one arm moving alone before it can vouch for a camera
MIN_LOG_RATIO = 0.4     # the arm's own camera must change about 1.5 times as much as the other one


def image_change(previous, current):
    """How much the picture changed, on small grey images."""
    return float(np.abs(current.astype(np.int16) - previous).mean())


def small_grey(rgb):
    import cv2
    return cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), (64, 36), interpolation=cv2.INTER_AREA)


def judge(changes, speeds):
    """changes: {'left': [...], 'right': [...]} per wrist camera. speeds: the same for the arms, sample for sample.
    Returns {verdict: ok|swapped|unknown, reason, votes: {arm: log ratio of its own camera's change to the other's}}."""
    cams = {k: np.asarray(v, float) for k, v in changes.items()}
    arms = {k: np.asarray(v, float) for k, v in speeds.items()}
    if set(cams) != {'left', 'right'} or set(arms) != {'left', 'right'}:
        return dict(verdict='unknown', reason='this needs both wrist cameras and both arms', votes={})
    if len({len(v) for v in (*cams.values(), *arms.values())}) != 1:
        raise ValueError('every series must have the same length')
    votes = {}
    for arm, other in (('left', 'right'), ('right', 'left')):
        alone = (arms[arm] > MOVING_RAD) & (arms[other] < STILL_RAD)
        if int(alone.sum()) >= MIN_ALONE:
            votes[arm] = round(float(np.log((cams[arm][alone].mean() + 1e-3) / (cams[other][alone].mean() + 1e-3))), 3)
    if not votes:
        return dict(verdict='unknown', reason='no arm moved alone for long enough: move one arm at a time, by hand or in a task', votes=votes)
    calls = {'ok' if v >= MIN_LOG_RATIO else 'swapped' if v <= -MIN_LOG_RATIO else None for v in votes.values()} - {None}
    if not calls:
        return dict(verdict='unknown', reason='neither camera clearly followed the arm: use larger moves, with something in view', votes=votes)
    if len(calls) > 1:
        return dict(verdict='unknown', reason='the two arms disagree about the cameras', votes=votes)
    if 'swapped' in calls:
        return dict(verdict='swapped', reason='the left and right wrist cameras look swapped: exchange their sources', votes=votes)
    return dict(verdict='ok', reason='each wrist camera follows its own arm', votes=votes)


def _arm_positions(station):
    with urllib.request.urlopen(f'{station}/supervisor/pose', timeout=1) as reply:
        measured = json.load(reply)['measured']
    return {arm: np.asarray(measured[arm]['positions_rad'], float) for arm in ('left', 'right') if measured.get(arm)}


def watch(socket_path, seconds, station='http://127.0.0.1:8091', rate_hz=SAMPLE_HZ, read_frame=None, read_arms=None, clock=time.monotonic):
    """Read-only: samples the camera service and the station for `seconds` while the operator (or a task) moves the arms."""
    from . import cameras
    def default_frame(role):
        import socket as sockets
        with sockets.socket(sockets.AF_UNIX) as conn:
            conn.settimeout(2); conn.connect(socket_path)
            cameras.send(conn, json.dumps(dict(operation='frame', role=role)).encode())
            payload = cameras.receive(conn, cameras.MAX_PACKET)
        if payload.startswith(b'{'):
            raise RuntimeError(json.loads(payload).get('error'))
        with np.load(__import__('io').BytesIO(payload), allow_pickle=False) as data:
            return data['rgb'].copy()
    read_frame, read_arms = read_frame or default_frame, read_arms or (lambda: _arm_positions(station))
    changes, speeds = dict(left=[], right=[]), dict(left=[], right=[])
    before, deadline = None, clock() + seconds
    while clock() < deadline:
        at = clock()
        now = dict(grey={c: small_grey(read_frame(c)) for c in ('left', 'right')}, arms=read_arms())
        if before is not None:
            for side in ('left', 'right'):
                changes[side].append(image_change(before['grey'][side], now['grey'][side]))
                speeds[side].append(float(np.abs(now['arms'][side] - before['arms'][side]).max()))
        before = now
        time.sleep(max(0., 1 / rate_hz - (clock() - at)))
    return judge(changes, speeds)
