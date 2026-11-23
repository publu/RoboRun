"""Import guard and heap rules for roborun-rt.

Garbage-collection pauses of a few hundred ms in a process that holds large vision libraries can end motion leases.
So the motor process never holds torch, transformers or cv2:
``install()`` refuses to start if any is already loaded and blocks later imports; ``loaded()`` lets ``roborun doctor``
and the tests prove it. MuJoCo stays allowed (the per-write mixed-arm check).
"""
from __future__ import annotations

import gc
import importlib.abc
import sys

FORBIDDEN = ("torch", "transformers", "cv2")


class ForbiddenImport(ImportError):
    """A forbidden heavy module was imported inside roborun-rt."""


class _Blocker(importlib.abc.MetaPathFinder):
    manifest_rt_guard = True

    def __init__(self, names):
        self.names = frozenset(names)

    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".", 1)[0] in self.names:
            raise ForbiddenImport(f"{fullname} is forbidden in roborun-rt: no torch, transformers or cv2 "
                                  "in the process that drives motors")
        return None


def loaded(names=FORBIDDEN) -> list[str]:
    """Forbidden top-level modules (and submodules) present in sys.modules."""
    return sorted(n for n in list(sys.modules) if n.split(".", 1)[0] in names)


def installed() -> bool:
    return any(getattr(f, "manifest_rt_guard", False) for f in sys.meta_path)


def install(names=FORBIDDEN) -> None:
    """Refuse to run if a forbidden module is already loaded; block any later import of one."""
    already = loaded(names)
    if already:
        raise RuntimeError("roborun-rt refuses to start: forbidden modules already loaded: " + ", ".join(already[:8]))
    if not installed():
        sys.meta_path.insert(0, _Blocker(names))


def uninstall() -> None:
    """Remove the blocker (tests only)."""
    sys.meta_path[:] = [f for f in sys.meta_path if not getattr(f, "manifest_rt_guard", False)]


def freeze_heap() -> int:
    """Collect once, then move every surviving object to the permanent generation (gc.freeze).

    Returns the number of frozen objects. Later collections then never traverse the start-up heap.
    """
    gc.collect()
    gc.freeze()
    return gc.get_freeze_count()


class NoCollect:
    """No automatic collection while a segment runs. Restores the previous gc state on exit."""

    def __enter__(self):
        self._was = gc.isenabled()
        gc.disable()
        return self

    def __exit__(self, *exc):
        if self._was:
            gc.enable()
        return False
