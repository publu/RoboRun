"""Roborun SDK: connect your arms (or a humanoid) to Roborun and drive them from your own code.

Public, stable names for programs:

- ``roborun_sdk.client``: ``Robot``, the typed client for the robot's local API on 127.0.0.1:8120 (tasks, Stop, camera
  frames, state), and ``RobotError`` / ``RobotUnavailable``.
- ``roborun_sdk.arms.base.StreamedArm``: the base class for your own arm driver; ``roborun_sdk.arms.open_arm`` loads one.
- ``roborun_sdk.errors.RoborunError``: the base class of every error the SDK raises to your code.

The rest is the robot side. What `roborun up` runs: ``roborun_sdk.robot`` (setup, start, stop), the arm drivers in
``roborun_sdk.arms``, the station and cameras, ``roborun_sdk.local_proxy`` (the local API), ``roborun_sdk.datasync``
(the history upload) and ``roborun_sdk.body`` (humanoids). ``roborun_sdk.cli`` is the ``roborun`` command.

``roborun_sdk.rt``, ``roborun_sdk.link`` and ``roborun_sdk.connectors`` are a signed-segment player with a local safety
kernel, its session client and its arm connectors, used by ``roborun doctor | run | rt`` and ``roborun stop --sim | --live``
(developer commands; ``roborun up`` does not use them). Their signing code (``rt.signing``, ``rt.canonical``) also
checks the humanoid's signed replies.

Importing this package performs no I/O.
"""

__version__ = "0.1.0"
SDK_NAME = "roborun-sdk/" + __version__
WIRE_VERSION = 1

__all__ = ['__version__', 'SDK_NAME', 'WIRE_VERSION', 'Robot', 'RobotError', 'RobotUnavailable', 'RoborunError']


def __getattr__(name):
    """Robot, RobotError, RobotUnavailable and RoborunError, imported on first use (importing this package stays I/O free)."""
    if name in ('Robot', 'RobotError', 'RobotUnavailable'):
        from . import client
        return getattr(client, name)
    if name == 'RoborunError':
        from .errors import RoborunError
        return RoborunError
    raise AttributeError(name)
