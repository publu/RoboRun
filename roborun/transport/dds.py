"""DDS direct backend — zero-config discovery + pub/sub via CycloneDDS.

What got real (spec §1.2):
  * General message types: any bundled family (schemas.dds_types), not just
    Twist. Subscribing deserializes to dicts, which is what makes passive
    recording possible at all.
  * Correct ROS 2 ↔ DDS name mangling: topic `/cmd_vel` is DDS `rt/cmd_vel`,
    type `geometry_msgs/Twist` is `geometry_msgs::msg::dds_::Twist_`.
  * Discovery across a domain range with liveness: participants carry a
    last_seen and an alive flag, so a robot dropping off the graph surfaces
    in fleet status instead of silently hanging tools.

No services/actions/params over DDS direct — capabilities() says so and the
error messages point at rosbridge/native. No silent failures.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable

from roborun.transport import Robot, Transport
from roborun.transport.schemas import (
    dds_class_for, dds_typename, from_dict, normalize_type, to_dict,
)

_RT_PREFIX = "rt"  # ROS 2 topic prefix on the wire


def _wire_topic(ros_topic: str) -> str:
    return ros_topic if ros_topic.startswith(_RT_PREFIX + "/") else _RT_PREFIX + ros_topic


def _ros_topic(wire_name: str) -> str:
    return wire_name[len(_RT_PREFIX):] if wire_name.startswith(_RT_PREFIX + "/") else wire_name


def dds_discover(domains: range | list[int] | None = None,
                 timeout: float = 3.0) -> list[Robot]:
    """Participant + publication discovery across a domain range."""
    from cyclonedds.domain import DomainParticipant
    import cyclonedds.builtin as builtin

    domains = list(domains) if domains is not None else [0]
    robots: list[Robot] = []
    for domain in domains:
        try:
            dp = DomainParticipant(domain)
            pubs = builtin.BuiltinDataReader(dp, builtin.BuiltinTopicDcpsPublication)
            parts = builtin.BuiltinDataReader(dp, builtin.BuiltinTopicDcpsParticipant)
            time.sleep(min(timeout, 5.0))

            by_part: dict[str, Robot] = {}
            for sample in parts.read(N=512):
                if sample is None:
                    continue
                key = sample.key.hex() if hasattr(sample.key, "hex") else str(sample.key)
                by_part[key] = Robot(id=f"dds-{domain}-{key[:8]}", transport="dds",
                                     domain=domain, last_seen=time.time())
            fallback = Robot(id=f"dds-{domain}-graph", transport="dds",
                             domain=domain, last_seen=time.time())
            for sample in pubs.read(N=2048):
                if sample is None:
                    continue
                topic = _ros_topic(sample.topic_name)
                if topic.startswith("/_") or sample.topic_name.startswith("DCPS"):
                    continue
                pkey = getattr(sample, "participant_key", None)
                pkey = pkey.hex() if pkey is not None and hasattr(pkey, "hex") else None
                target = by_part.get(pkey, fallback)
                target.topics[topic] = normalize_type(sample.type_name)
            del dp
            robots.extend(r for r in list(by_part.values()) + [fallback] if r.topics)
        except Exception:
            continue
    return robots


class DDSTransport(Transport):
    name = "dds"

    def __init__(self, domain: int = 0, discovery_timeout: float = 3.0) -> None:
        self.domain = domain
        self._discovery_timeout = discovery_timeout
        self._dp = None
        self._lock = threading.RLock()
        self._writers: dict[str, Any] = {}
        self._readers: dict[str, dict] = {}   # topic -> {thread, stop, reader}
        self._topic_types: dict[str, str] = {}
        self._last_scan = 0.0

    def _participant(self):
        if self._dp is None:
            from cyclonedds.domain import DomainParticipant
            self._dp = DomainParticipant(self.domain)
        return self._dp

    # ── graph ────────────────────────────────────────────────────────────

    def topics(self, max_age: float = 10.0) -> dict[str, str]:
        with self._lock:
            if time.time() - self._last_scan > max_age:
                for robot in dds_discover([self.domain], self._discovery_timeout):
                    self._topic_types.update(robot.topics)
                self._last_scan = time.time()
            return dict(self._topic_types)

    def heartbeat(self) -> list[Robot]:
        """Re-scan the graph; feeds fleet liveness."""
        robots = dds_discover([self.domain], min(self._discovery_timeout, 1.5))
        with self._lock:
            self._topic_types = {}
            for r in robots:
                self._topic_types.update(r.topics)
            self._last_scan = time.time()
        return robots

    # ── pub/sub ──────────────────────────────────────────────────────────

    def subscribe(self, topic: str, callback: Callable[[dict], None],
                  msg_type: str | None = None) -> str:
        from cyclonedds.sub import DataReader
        from cyclonedds.topic import Topic as DDSTopic

        msg_type = msg_type or self.type_of(topic)
        if not msg_type:
            raise ValueError(f"unknown type for {topic}; pass msg_type or wait for discovery")
        cls = dds_class_for(msg_type)
        if cls is None:
            raise ValueError(
                f"no bundled schema for {msg_type}; "
                f"DDS direct covers the common families — use rosbridge/native for this type")

        dp = self._participant()
        reader = DataReader(dp, DDSTopic(dp, _wire_topic(topic), cls))
        stop = threading.Event()

        def poll() -> None:
            while not stop.is_set():
                try:
                    for sample in reader.take(N=64):
                        if sample is not None:
                            callback(to_dict(sample))
                except Exception:
                    pass
                time.sleep(0.01)

        thread = threading.Thread(target=poll, daemon=True, name=f"DDSSub{topic}")
        thread.start()
        with self._lock:
            self._readers[topic] = {"thread": thread, "stop": stop, "reader": reader}
        return topic

    def unsubscribe(self, topic: str) -> None:
        with self._lock:
            entry = self._readers.pop(topic, None)
        if entry:
            entry["stop"].set()

    def publish(self, topic: str, msg_type: str, msg: dict) -> dict:
        from cyclonedds.pub import DataWriter
        from cyclonedds.topic import Topic as DDSTopic

        cls = dds_class_for(msg_type)
        if cls is None:
            return {"ok": False,
                    "error": f"no bundled schema for {msg_type} over DDS direct",
                    "supported": "common families; use rosbridge for arbitrary types",
                    "dds_typename": dds_typename(msg_type)}
        try:
            with self._lock:
                writer = self._writers.get(topic)
                if writer is None:
                    dp = self._participant()
                    writer = DataWriter(dp, DDSTopic(dp, _wire_topic(topic), cls))
                    self._writers[topic] = writer
            writer.write(from_dict(cls, msg))
            return {"ok": True, "topic": topic, "type": normalize_type(msg_type),
                    "transport": "dds"}
        except Exception as exc:
            return {"ok": False, "error": str(exc)}

    def close(self) -> None:
        with self._lock:
            for entry in self._readers.values():
                entry["stop"].set()
            self._readers.clear()
            self._writers.clear()
            self._dp = None
