"""K8 interlock bits, per arm, persisted on disk, kept at most 12 h.

- ``held``: stored with the jaw value and the tool height at grasp. Set conservatively by the kernel whenever a
  close ends on a contact trigger (it can only block motion); confirmed or cleared by the API (``set_interlock``),
  cleared by a verified release or the operator's "gripper empty" attestation.
- ``pending_release``: only a release-escape segment is accepted (none in this SDK version, so motion is refused).
While ``held``: no jaw-opening Home, no Reset without confirmation, the jaw opens only in an accepted release, and
link loss never opens a jaw. A part-closed jaw is not proof of an object.

Seeding: the robot runtime keeps the same facts in its jaw memory file (per arm
{jaw, task, t, z, name, release_pending}, 12 h). ``seed_from_jaw_memory`` reads it read-only and sets the bits
conservatively.
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path

from ..datatypes import OperatorConfirm, Refused

MAX_AGE_S = 12 * 3600.0


class Interlocks:
    def __init__(self, path: str | Path | None, arms, *, wall=time.time):
        self.path = None if path is None else Path(path)
        self.wall = wall
        self.bits = {arm: {"held": None, "pending_release": None} for arm in arms}
        if self.path is not None:
            self.load()

    def load(self) -> None:
        try:
            raw = json.loads(self.path.read_text())
        except (OSError, ValueError):
            return
        now = self.wall()
        for arm, entry in (raw or {}).items():
            if arm not in self.bits or not isinstance(entry, dict):
                continue
            for key in ("held", "pending_release"):
                rec = entry.get(key)
                if isinstance(rec, dict) and now - float(rec.get("wall_t", 0)) <= MAX_AGE_S:
                    self.bits[arm][key] = rec

    def save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), prefix=".interlocks-", suffix=".json")
        with os.fdopen(fd, "w") as fh:
            json.dump(self.bits, fh, indent=1, sort_keys=True)
        os.replace(tmp, self.path)

    def held(self, arm: str) -> dict | None:
        return self.bits[arm]["held"]

    def pending_release(self, arm: str) -> dict | None:
        return self.bits[arm]["pending_release"]

    def set_held(self, arm: str, *, jaw: float | None, tool_z_m: float | None, evidence: str, source: str) -> dict:
        rec = {"jaw": jaw, "tool_z_m": tool_z_m, "evidence": evidence, "source": source, "wall_t": self.wall()}
        self.bits[arm]["held"] = rec
        self.save()
        return rec

    def set_pending_release(self, arm: str, *, evidence: str, source: str) -> dict:
        rec = {"evidence": evidence, "source": source, "wall_t": self.wall()}
        self.bits[arm]["pending_release"] = rec
        self.save()
        return rec

    def clear(self, arm: str, *, reason: str, confirm: OperatorConfirm | None = None, measured_jaw: float | None = None,
              open_value: float | None = None, open_tol: float = 0.03) -> dict:
        """'verified_release' needs the measured jaw at open; 'operator_attested' needs an operator confirmation;
        'api_confirmed' comes from a signed control after hosted evidence."""
        if reason == "verified_release":
            if measured_jaw is None or open_value is None or abs(measured_jaw - open_value) > open_tol:
                raise Refused("release_not_verified", "the measured jaw has not reached open")
        elif reason == "operator_attested":
            if not isinstance(confirm, OperatorConfirm) or not confirm.operator:
                raise Refused("operator_confirmation_required", "'gripper empty' is an operator attestation")
        elif reason != "api_confirmed":
            raise Refused("malformed", "unknown interlock clear reason")
        before = dict(self.bits[arm])
        self.bits[arm] = {"held": None, "pending_release": None}
        self.save()
        return {"arm": arm, "cleared": before, "reason": reason,
                "operator": None if confirm is None else confirm.operator}

    def seed_from_jaw_memory(self, path: str | Path) -> list[str]:
        """Read the robot runtime's jaw memory (never written here). Conservative: any recent entry sets the bit."""
        try:
            raw = json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return []
        seeded = []
        now = self.wall()
        for arm, entry in (raw or {}).items():
            if arm not in self.bits or not isinstance(entry, dict):
                continue
            pending = entry.get("release_pending")
            fresh = now - float(entry.get("t") or 0.0) <= MAX_AGE_S
            if isinstance(pending, dict) and pending:
                self.bits[arm]["pending_release"] = {"evidence": "runtime jaw memory release_pending",
                                                     "source": "jaw_memory", "wall_t": now}
                seeded.append(f"{arm}: pending_release")
            if fresh and entry.get("jaw") is not None:
                self.bits[arm]["held"] = {"jaw": float(entry["jaw"]), "tool_z_m": entry.get("z"),
                                          "evidence": f"runtime jaw memory: {entry.get('name') or 'object'}",
                                          "source": "jaw_memory", "wall_t": now}
                seeded.append(f"{arm}: held")
        if seeded:
            self.save()
        return seeded

    def public(self) -> dict:
        return {arm: {"held": b["held"] is not None, "pending_release": b["pending_release"] is not None,
                      **({"held_jaw": b["held"].get("jaw")} if b["held"] else {})} for arm, b in self.bits.items()}
