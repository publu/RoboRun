"""K2 link watchdog.

LINKED: an API message within the late threshold. LATE (0.25 s QUIC / 0.5 s WSS): nothing new is accepted; the
running segment continues under every local check. LOST (0.5 s / 1.0 s): the runtime applies the segment's
``on_link_loss`` (hold), revokes grants, latches ``link_lost`` and sets a scene-change mark. HELD: after the hold.
BACK: only a new session (admit) clears the latch; nothing resumes and old plans are refused by the mark.
A signed ``goaway`` control is an orderly link loss: the same hold and latch at once, never an unsupervised state.
There is no automatic retreat at any duration.
"""
from __future__ import annotations

from ..datatypes import Event


class LinkWatchdog:
    def __init__(self, *, transport: str = "wss", late_after_s: float = 0.5, lost_after_s: float = 1.0):
        self.transport = transport
        self.late_after_s = late_after_s
        self.lost_after_s = lost_after_s
        self.state = "NONE"            # no session yet: nothing is accepted
        self.session: str | None = None
        self.last_msg_t: float | None = None
        self.latched_lost = False
        self.lost_at: float | None = None

    def configure(self, *, transport: str, late_after_s: float, lost_after_s: float) -> None:
        if not 0 < late_after_s <= lost_after_s:
            raise ValueError("late threshold must be positive and not above the lost threshold")
        self.transport, self.late_after_s, self.lost_after_s = transport, late_after_s, lost_after_s

    def session_started(self, session: str, now: float) -> list[Event]:
        events = []
        if self.latched_lost or self.state in ("LOST", "HELD"):
            events.append(Event("link_back", now, {"session": session, "previous": self.session,
                                                     "lost_for_s": None if self.lost_at is None else round(now - self.lost_at, 3)}))
        self.session = session
        self.state = "LINKED"
        self.last_msg_t = now
        self.latched_lost = False
        self.lost_at = None
        return events

    def message(self, now: float) -> None:
        """Any message from the API (heartbeats included). LOST/HELD need a new session, not a heartbeat."""
        if self.last_msg_t is None or now >= self.last_msg_t:
            self.last_msg_t = now
        if self.state == "LATE":
            self.state = "LINKED"

    def poll(self, now: float) -> list[Event]:
        if self.state not in ("LINKED", "LATE") or self.last_msg_t is None:
            return []
        gap = now - self.last_msg_t
        if gap >= self.lost_after_s:
            self.state = "LOST"
            self.latched_lost = True
            self.lost_at = now
            return [Event("link_lost", now, {"gap_s": round(gap, 3), "transport": self.transport, "latched": True})]
        if gap >= self.late_after_s and self.state == "LINKED":
            self.state = "LATE"
            return [Event("link_late", now, {"gap_s": round(gap, 3)})]
        return []

    def goaway(self, now: float) -> list[Event]:
        """The API ends the session (signed control ``goaway``): treated as link loss now, latched until a new
        session, so a running segment is never left without link supervision."""
        self.state = "LOST"
        self.latched_lost = True
        self.lost_at = now
        return [Event("link_lost", now, {"why": "goaway", "transport": self.transport, "latched": True})]

    def mark_held(self) -> None:
        if self.state == "LOST":
            self.state = "HELD"

    @property
    def accepting(self) -> bool:
        return self.state == "LINKED" and not self.latched_lost

    def public(self, now: float) -> dict:
        return {"state": self.state, "session": self.session, "transport": self.transport,
                "gap_s": None if self.last_msg_t is None else round(now - self.last_msg_t, 3),
                "latched_lost": self.latched_lost}
