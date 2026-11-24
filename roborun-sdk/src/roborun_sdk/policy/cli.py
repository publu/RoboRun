"""`roborun policy ...`: run a policy on this robot as a Roborun task.

    roborun policy providers [--json]
    roborun policy check --provider P [--model M] [--arms left|right|both] [--option k=v ...]
    roborun policy run --provider P [--model M] --prompt "..." [--arms both] [--seconds 60] [--max-tool-speed 0.25]
                       [--option k=v ...] [--wait-ready 2m] [--shadow] [--no-dataset] [--json] [--yes]
    roborun policy runs [--last 10] [--json]
    roborun policy consent [--revoke PROVIDER[:MODEL]] [--json]

Exit codes: 0 done or stopped, 1 failed, 2 refused (nothing sent, nothing moved), 3 the model was ready too late to start
moving on its own (run again).
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path


def _duration(text):
    m = re.fullmatch(r'\s*([0-9]+(?:\.[0-9]+)?)\s*(s|m|h)?\s*', str(text))
    if not m:
        raise SystemExit(f'--wait-ready {text!r}: a duration like 90s, 2m or 30m')
    return float(m.group(1)) * {'s': 1, 'm': 60, 'h': 3600, None: 1}[m.group(2)]


def _arms(text):
    return None if text in (None, '') else text


HUMAN = {
    'policy_start': lambda e: f"policy: {e.get('provider')}" + (f" {e['model']}" if e.get('model') else '') +
                              f" on {', '.join(e.get('arms') or [])}" + (' (shadow: nothing moves)' if e.get('shadow') else ''),
    'policy_consent': lambda e: f"consent: {e.get('how')}",
    'policy_consent_text': lambda e: e.get('text'),
    'policy_warming': lambda e: e.get('text'),
    'policy_ready': lambda e: (f"the model is ready ({e['after_s']} s)" if 'after_s' in e
                               else f"the model became ready after {e.get('late_s')} s"),
    'policy_lease': lambda e: f"moving: lease {str(e.get('lease'))[:8]}, tool speed cap {e.get('tool_speed_m_s')} m/s",
    'policy_refused': lambda e: f"chunk refused by the {e.get('by')}: {e.get('why') or e.get('code')}",
    'policy_hold': lambda e: f"holding: {e.get('why')}",
    'policy_notice': lambda e: f"note: {e.get('text')}",
    'policy_end': lambda e: (f"shadow run: {e.get('chunks')} chunks, largest seam {e.get('seam_rad_max')} rad, fastest row "
                             f"{e.get('joint_speed_rad_s_max')} rad/s, would refuse {e.get('would_refuse') or 'none'}"
                             if e.get('status') == 'shadow' else f"{e.get('status')}: {e.get('message')}"),
}


def _printer(as_json):
    def show(event):
        if as_json:
            print(json.dumps(event, default=str), flush=True)
            return
        fn = HUMAN.get(event.get('event'))
        if fn is not None:
            text = fn(event)
            if text:
                print(text, flush=True)
    return show


def cmd_providers(a):
    from .registry import providers
    rows = providers()
    if a.json:
        print(json.dumps([r.__dict__ for r in rows], indent=1))
        return 0
    for r in rows:
        state = 'ready' if r.available else (r.note or 'not available')
        print(f'{r.name:12s} {r.source:12s} {r.version or "-":28s} {state}')
    print('your own: --provider my_policy.py:MyPolicy or mypackage.module:MyPolicy (see roborun docs building, '
          '"Run your own policy")')
    return 0


def _common(a):
    from .registry import parse_options
    return dict(model=a.model, arms=_arms(a.arms), options=parse_options(a.option))


def cmd_check(a):
    from . import PolicyRefused, check
    try:
        out = check(a.provider, **_common(a))
    except PolicyRefused as exc:
        if a.json:
            print(json.dumps(dict(ok=False, refused=str(exc))))
        else:
            print(f'refused: {exc}', file=sys.stderr)
        return 2
    if a.json:
        print(json.dumps(out, indent=1, default=str))
    else:
        print(f"ok: {out['provider']} ({out['version']}) can drive {', '.join(out['arms'])}"
              + (f"; cameras {', '.join(sorted(set(out['cameras'].values())))}" if out['cameras'] else '')
              + ('; sends frames and state off this computer (consent is asked before a run)' if out['remote'] else '')
              + (f"; key: {out['key']}" if out.get('key') else ''))
    return 0


class _stop_on_signals:
    """SIGTERM and SIGHUP (a closed terminal, systemd, kill) end the run like Ctrl-C: the lease ends, the arms hold, and the
    task record, recording and episode are closed. A second signal, or one before the run's record exists, interrupts."""
    NAMES = ('SIGTERM', 'SIGHUP')

    def __init__(self, runner):
        self.runner, self.old = runner, {}

    def __enter__(self):
        import signal
        import threading
        if threading.current_thread() is not threading.main_thread():
            return self
        for name in self.NAMES:
            sig = getattr(signal, name, None)
            if sig is not None:
                try:
                    self.old[sig] = signal.signal(sig, self._handle)
                except (OSError, ValueError):
                    pass
        return self

    def _handle(self, signum, frame):
        import signal
        if self.runner._stopping.is_set() or self.runner.record is None:
            raise KeyboardInterrupt
        self.runner.stop(signal.Signals(signum).name)

    def __exit__(self, *exc):
        import signal
        for sig, old in self.old.items():
            try:
                signal.signal(sig, old)
            except (OSError, ValueError):
                pass
        return False


def cmd_run(a):
    from . import PolicyNotStarted, PolicyRefused
    from .consent import terminal_ask
    from .runner import PolicyRunner, WAIT_READY_MAX_S
    wait = _duration(a.wait_ready)
    if wait > WAIT_READY_MAX_S:
        raise SystemExit(f'--wait-ready is at most {WAIT_READY_MAX_S / 60:.0f}m')
    show = _printer(a.json)
    ask = None if a.json else terminal_ask()
    try:
        runner = PolicyRunner(a.provider, prompt=a.prompt or '', seconds=a.seconds, consent=None, yes=a.yes, ask=ask,
                              shadow=a.shadow, dataset=not a.no_dataset, max_tool_speed=a.max_tool_speed, wait_ready=wait,
                              on_event=show, **_common(a))
        if not a.json:
            print(f'task: {runner.id}', flush=True)
            try:
                from ..robot import viewer_url
                url = viewer_url()
                if url and not a.shadow:
                    print(f'viewer: {url}', flush=True)
            except Exception:
                pass
        with _stop_on_signals(runner):
            result = runner.run()
    except PolicyRefused as exc:
        show(dict(event='policy_end', status='refused', message=str(exc)))
        return 2
    except PolicyNotStarted as exc:
        if not a.json:
            print(str(exc))
        return 3
    except KeyboardInterrupt:                        # before anything moved (a running run ends itself on Ctrl-C)
        show(dict(event='policy_end', status='stopped', message='Stopped before anything moved'))
        return 130
    if a.shadow:
        if a.json:
            print(json.dumps(dict(event='policy_shadow', **result.__dict__), default=str))
        else:
            print(f'saved: {result.path}')
        return 0
    if a.json:
        print(json.dumps(dict(event='policy_result', task_id=result.task_id, status=result.status,
                              stop_reason=result.stop_reason, summary=result.summary), default=str), flush=True)
    return 0 if result.status in ('done', 'stopped') else 1


def runs(last=10, runtime=None):
    """The newest policy runs: from the task records plus the local summaries."""
    from ..robot import CONFIG
    from .record import robot_runtime
    if runtime is None:
        try:
            config = json.loads(CONFIG.read_text())
        except (OSError, ValueError):
            config = None
        runtime = robot_runtime(config)
    root = Path(runtime) / 'log'
    try:
        from .record import close_orphans
        close_orphans(root.parent)                       # a run whose process was killed is listed as failed, not running
    except Exception:
        pass
    out = []
    for path in sorted((root / 'tasks').glob('*.json'), key=lambda p: p.stat().st_mtime, reverse=True):
        try:
            rec = json.loads(path.read_text())
        except (OSError, ValueError):
            continue
        if rec.get('kind') != 'policy':
            continue
        try:
            summary = json.loads((root / 'policy' / rec['id'] / 'summary.json').read_text())
        except (OSError, ValueError, KeyError):
            summary = {}
        pol = rec.get('policy') or {}
        out.append(dict(id=rec.get('id'), created=rec.get('created'), provider=pol.get('provider'), model=pol.get('model'),
                        status=rec.get('status'), stop_reason=rec.get('stop_reason'),
                        chunks=(rec.get('safety') or {}).get('chunks_played'), rtt_ms=summary.get('rtt_ms')))
        if len(out) >= last:
            break
    return out


def cmd_runs(a):
    rows = runs(a.last)
    if a.json:
        print(json.dumps(rows, indent=1, default=str))
        return 0
    if not rows:
        print('no policy runs yet')
    for r in rows:
        rtt = r.get('rtt_ms') or {}
        print(f"{r['id'][:12]} {str(r['created'])[:19]} {r['provider']}{':' + r['model'] if r.get('model') else ''} "
              f"{r['status']} ({r['stop_reason']}) chunks {r['chunks']} rtt p50/p90 {rtt.get('p50')}/{rtt.get('p90')} ms")
    return 0


def cmd_consent(a):
    from . import consent
    if a.revoke:
        gone = consent.revoke(a.revoke)
        print(json.dumps(dict(revoked=gone)) if a.json else (f"revoked: {', '.join(gone)}" if gone else 'nothing to revoke'))
        return 0
    return consent.show(a.json)


def add_parser(sub):
    p = sub.add_parser('policy', help='run a policy (your own model, or a provider\'s) on this robot as a Roborun task')
    ps = p.add_subparsers(dest='policy_cmd', metavar='providers|check|run|runs|consent', required=True)
    s = ps.add_parser('providers', help='the installed policy providers')
    s.add_argument('--json', action='store_true')
    s.set_defaults(func=cmd_providers)

    def common(s):
        s.add_argument('--provider', required=True, help='hold, replay, dreamlabs, file.py:Class, module:Class or an installed '
                                                         'provider\'s name')
        s.add_argument('--model', help="the provider's model, if it has several")
        s.add_argument('--arms', choices=('left', 'right', 'both'), help='the arms it may move (default: all it drives)')
        s.add_argument('--option', action='append', default=[], metavar='K=V', help="the provider's own options")
        s.add_argument('--json', action='store_true')
    s = ps.add_parser('check', help='offline: does this provider fit this robot? Nothing is sent and nothing moves')
    common(s)
    s.set_defaults(func=cmd_check)
    s = ps.add_parser('run', help='run one episode as a Roborun task (Ctrl-C or roborun stop ends it)')
    common(s)
    s.add_argument('--prompt', help='what the arms should do')
    s.add_argument('--seconds', type=float, help='how long (default 60, at most 595; shadow: 20)')
    s.add_argument('--max-tool-speed', type=float, help='m/s: lower the tool speed cap for this run (never above robot.yaml)')
    s.add_argument('--wait-ready', default='2m', help='how long the model may warm up (default 2m, at most 30m)')
    s.add_argument('--shadow', action='store_true', help='ask for chunks against live observations but move nothing')
    s.add_argument('--no-dataset', action='store_true', help='no dataset episode for this run (it can only turn episodes off; robot.json decides whether the robot keeps a dataset)')
    s.add_argument('--yes', action='store_true', help='consent for this run only (not stored)')
    s.set_defaults(func=cmd_run)
    s = ps.add_parser('runs', help='past policy runs')
    s.add_argument('--last', type=int, default=10)
    s.add_argument('--json', action='store_true')
    s.set_defaults(func=cmd_runs)
    s = ps.add_parser('consent', help='what you agreed to send to remote providers; --revoke to forget it')
    s.add_argument('--revoke', metavar='PROVIDER[:MODEL]')
    s.add_argument('--json', action='store_true')
    s.set_defaults(func=cmd_consent)
