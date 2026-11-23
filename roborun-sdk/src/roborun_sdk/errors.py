"""The errors the SDK raises to programs that import it. Catch RoborunError to catch them all."""
from __future__ import annotations


class RoborunError(Exception):
    """Base class of every error roborun_sdk raises to your code."""


class ArmLoadError(RoborunError, SystemExit):
    """An arm driver could not be found or loaded (open_arm). It is also a SystemExit, so the `roborun` commands that load
    a driver still end with just its message; a program catches it as RoborunError (or Exception)."""
