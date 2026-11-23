"""Session transports: WSS (msgpack frames, link lost after 1.0 s, speed grants <= 0.10 m/s).

``MemoryTransport.pair()`` gives two connected in-process ends for tests and the local sim API.
``WebSocketTransport`` is the WSS client (``websockets`` sync API, imported lazily; roborun-link only).
Bulk (frames, traces) never rides this connection: see ``upload``.
"""
from __future__ import annotations

import queue
import ssl
import threading
from typing import Protocol


class TransportClosed(ConnectionError):
    pass


class Transport(Protocol):
    def send(self, data: bytes) -> None: ...
    def recv(self, timeout: float) -> bytes | None: ...
    def close(self) -> None: ...


class MemoryTransport:
    def __init__(self, inbox: queue.Queue, outbox: queue.Queue, closed: threading.Event):
        self.inbox, self.outbox, self.closed = inbox, outbox, closed
        self.sent = 0

    @classmethod
    def pair(cls):
        a, b, closed = queue.Queue(), queue.Queue(), threading.Event()
        return cls(a, b, closed), cls(b, a, closed)

    def send(self, data: bytes) -> None:
        if self.closed.is_set():
            raise TransportClosed("closed")
        self.sent += 1
        self.outbox.put(bytes(data))

    def recv(self, timeout: float) -> bytes | None:
        if self.closed.is_set() and self.inbox.empty():
            raise TransportClosed("closed")
        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty:
            if self.closed.is_set():
                raise TransportClosed("closed") from None
            return None

    def close(self) -> None:
        self.closed.set()


class WebSocketTransport:
    """WSS client. ``cert``/``key``: the device certificate (mTLS); ``ca``: the API's CA bundle."""

    def __init__(self, url: str, *, cert: str | None = None, key: str | None = None, ca: str | None = None,
                 open_timeout: float = 5.0):
        if not url.startswith("wss://") and not url.startswith("ws://127.0.0.1"):
            raise ValueError("the session runs over wss:// (ws:// only to loopback for local testing)")
        from websockets.sync.client import connect  # roborun-link only
        ctx = None
        if url.startswith("wss://"):
            ctx = ssl.create_default_context(cafile=ca)
            ctx.minimum_version = ssl.TLSVersion.TLSv1_3
            if cert:
                ctx.load_cert_chain(cert, key)
        self._ws = connect(url, ssl=ctx, open_timeout=open_timeout, max_size=4 * 1024 * 1024, compression=None)

    def send(self, data: bytes) -> None:
        try:
            self._ws.send(data)
        except Exception as exc:
            raise TransportClosed(str(exc)) from exc

    def recv(self, timeout: float) -> bytes | None:
        try:
            msg = self._ws.recv(timeout=timeout)
        except TimeoutError:
            return None
        except Exception as exc:
            raise TransportClosed(str(exc)) from exc
        return msg if isinstance(msg, bytes) else msg.encode()

    def close(self) -> None:
        try:
            self._ws.close()
        except Exception:
            pass
