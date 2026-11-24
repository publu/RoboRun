"""Run a policy (your own model, or a provider's) on this robot, as a Roborun task.

New in this release; it may change within 0.1, and it is not on the stable surface yet.

    from roborun_sdk import policy

    result = policy.run('hold', prompt='self-test', arms='left', seconds=5)        # blocks; a RunResult
    result.task_id, result.status, result.stop_reason, result.summary

    policy.run(MyPolicy(), prompt='wave', arms='left', seconds=20)                 # your own Provider object
    policy.run('my_policy.py:MyPolicy', prompt='wave', options={'amplitude': 0.1})
    policy.run('provider', model='its/model', prompt='fold the towel', consent=True)   # a remote provider: consent
    policy.run('provider', model=..., prompt=..., shadow=True)                     # moves nothing; a ShadowReport
    policy.check('provider', model=..., arms='both')                               # needs() only, offline; a dict
    policy.providers()                                                             # [ProviderEntry]

    run = policy.start(...)          # the same, in the background: run.stop(), run.wait(), run.events()

Roborun stays in charge while a policy runs: the station plays the policy's chunks through every limit (the per-tick clip,
the seam and age checks, the joint and tool speed limits, your rules file), the arms hold when the policy is late, and Stop
(the page, the viewer, `roborun stop`) ends the run at once. Every run is a task with a record on this computer, and a
recording and a dataset episode when this robot records its tasks. `policy.run` never asks on the terminal: a remote
provider needs consent=True (this run) or a consent stored by `roborun policy run`, and a model that became ready after
the start-motion limit raises PolicyNotStarted.

Standard library at import (Windows too).
"""
from __future__ import annotations

import threading

from .provider import (ArmInfo, ArmState, Chunk, Frame, Needs, Observation, PolicyError, PolicyNotStarted, PolicyRefused,
                       Provider, ProviderError, RunSpec, plain_needs)
from .registry import ProviderEntry, providers

__all__ = ['run', 'start', 'check', 'providers', 'PolicyRun', 'Provider', 'ProviderError', 'PolicyError', 'PolicyRefused',
           'PolicyNotStarted', 'ArmInfo', 'ArmState', 'Chunk', 'Frame', 'Needs', 'Observation', 'RunSpec', 'ProviderEntry',
           'plain_needs']


def _runner(provider, *, model=None, prompt='', arms=None, seconds=None, options=None, consent=None, shadow=False,
            dataset=True, max_tool_speed=None, wait_ready=None, **kw):
    from .runner import WAIT_READY_S, PolicyRunner
    return PolicyRunner(provider, prompt=prompt, arms=arms, seconds=seconds, model=model, options=options, consent=consent,
                        ask=None, shadow=shadow, dataset=dataset, max_tool_speed=max_tool_speed,
                        wait_ready=WAIT_READY_S if wait_ready is None else wait_ready, **kw)


def run(provider, *, model=None, prompt='', arms=None, seconds=None, options=None, consent=None, shadow=False, dataset=True,
        max_tool_speed=None, wait_ready=None, **kw):
    """One policy run, blocking. provider: a name ('hold', 'replay', 'file.py:Class', 'module:Class', an entry
    point) or a Provider object. Returns a RunResult (ShadowReport with shadow=True). Raises PolicyRefused before anything
    was sent or moved, PolicyNotStarted when the model was ready too late to start moving on its own."""
    return _runner(provider, model=model, prompt=prompt, arms=arms, seconds=seconds, options=options, consent=consent,
                   shadow=shadow, dataset=dataset, max_tool_speed=max_tool_speed, wait_ready=wait_ready, **kw).run()


def check(provider, *, model=None, arms=None, options=None, prompt='check', **kw):
    """needs() and the match against this robot, offline: nothing is sent, nothing is leased, no consent is asked."""
    return _runner(provider, model=model, prompt=prompt, arms=arms, options=options, **kw).check()


class PolicyRun:
    """A run in the background (policy.start)."""

    def __init__(self, runner):
        self.runner = runner
        self.task_id = runner.id
        self.result = None
        self.error = None
        self._thread = threading.Thread(target=self._go, name='policy-run', daemon=True)
        self._thread.start()

    def _go(self):
        try:
            self.result = self.runner.run()
        except BaseException as exc:              # noqa: BLE001 - handed to wait()
            self.error = exc

    def stop(self):
        """End the run as an ordinary stop: the arms settle where they are and the record says stopped."""
        self.runner.stop('program')

    def wait(self, timeout=None):
        self._thread.join(timeout)
        if self._thread.is_alive():
            return None
        if self.error is not None:
            raise self.error
        return self.result

    def events(self, timeout=None):
        """The run's events as they happen (dicts: event, run, ...), until policy_end."""
        import queue
        while True:
            try:
                item = self.runner.events.get(timeout=timeout)
            except queue.Empty:
                return
            yield item
            if item.get('event') == 'policy_end':
                return


def start(provider, **kw):
    """policy.run's arguments; returns a PolicyRun at once."""
    return PolicyRun(_runner(provider, **kw))
