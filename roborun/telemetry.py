"""Telemetry bus — collects data from sim/rosbridge/webcam and broadcasts via WebSocket.

The bus stores a rolling window per (robot_id, channel) and pushes deltas
to all connected WebSocket clients. Clients receive full history on connect.
"""

from __future__ import annotations

import asyncio
import json
import threading
import time
from collections import defaultdict, deque
from typing import Any

RING_SIZE = 600
WS_PORT = 8766


class TelemetryBus:
    _instance: TelemetryBus | None = None
    _lock = threading.Lock()

    def __init__(self) -> None:
        self._channels: dict[str, deque[dict]] = defaultdict(lambda: deque(maxlen=RING_SIZE))
        self._bus_lock = threading.Lock()
        self._ws_clients: list[asyncio.Queue] = []
        self._ws_lock = threading.Lock()

    @classmethod
    def get(cls) -> TelemetryBus:
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def push(self, robot_id: str, channel: str, data: dict[str, Any]) -> None:
        key = f"{robot_id}/{channel}"
        entry = {"t": time.time(), "robot_id": robot_id, "channel": channel, **data}
        with self._bus_lock:
            self._channels[key].append(entry)
        self._broadcast(entry)

    def get_history(self, robot_id: str | None = None, channel: str | None = None,
                    limit: int = 200) -> list[dict]:
        with self._bus_lock:
            if robot_id and channel:
                key = f"{robot_id}/{channel}"
                return list(self._channels[key])[-limit:]
            results = []
            for key, ring in self._channels.items():
                if robot_id and not key.startswith(f"{robot_id}/"):
                    continue
                if channel and not key.endswith(f"/{channel}"):
                    continue
                results.extend(list(ring)[-limit:])
            results.sort(key=lambda e: e.get("t", 0))
            return results[-limit:]

    def get_latest(self, robot_id: str = "local") -> dict[str, Any]:
        snapshot = {}
        with self._bus_lock:
            for key, ring in self._channels.items():
                if key.startswith(f"{robot_id}/") and ring:
                    ch = key.split("/", 1)[1]
                    snapshot[ch] = dict(ring[-1])
        return snapshot

    def _broadcast(self, entry: dict) -> None:
        msg = json.dumps(entry)
        with self._ws_lock:
            dead = []
            for q in self._ws_clients:
                try:
                    q.put_nowait(msg)
                except asyncio.QueueFull:
                    dead.append(q)
            for q in dead:
                self._ws_clients.remove(q)

    def register_client(self, q: asyncio.Queue) -> None:
        with self._ws_lock:
            self._ws_clients.append(q)

    def unregister_client(self, q: asyncio.Queue) -> None:
        with self._ws_lock:
            try:
                self._ws_clients.remove(q)
            except ValueError:
                pass


async def _ws_handler(ws: Any, bus: TelemetryBus) -> None:
    q: asyncio.Queue = asyncio.Queue(maxsize=256)
    bus.register_client(q)
    try:
        history = bus.get_history(limit=200)
        await ws.send(json.dumps({"type": "history", "data": history}))
        while True:
            msg = await q.get()
            await ws.send(msg)
    except Exception:
        pass
    finally:
        bus.unregister_client(q)


async def _run_ws_server(bus: TelemetryBus) -> None:
    try:
        import websockets
        async with websockets.serve(
            lambda ws: _ws_handler(ws, bus),
            "127.0.0.1", WS_PORT,
            ping_interval=20, ping_timeout=10,
        ):
            await asyncio.Future()
    except ImportError:
        import asyncio as _a
        server = await _a.start_server(
            lambda r, w: _fallback_handler(r, w, bus),
            "127.0.0.1", WS_PORT,
        )
        async with server:
            await server.serve_forever()


async def _fallback_handler(reader: Any, writer: Any, bus: TelemetryBus) -> None:
    writer.close()


def start_ws_server() -> None:
    bus = TelemetryBus.get()

    def _run() -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(_run_ws_server(bus))
        except Exception:
            pass

    t = threading.Thread(target=_run, daemon=True, name="TelemetryWS")
    t.start()
