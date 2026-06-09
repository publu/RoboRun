"""Lightweight route registry for RoboRun's HTTP server.

Routes are plain functions decorated with @get or @post. The Handler
class calls dispatch(method, path, handler) to find a match.
"""
from __future__ import annotations

import json
import re
from http.server import SimpleHTTPRequestHandler
from typing import Any, Callable

_GET_ROUTES: list[tuple[re.Pattern, Callable]] = []
_POST_ROUTES: list[tuple[str, Callable]] = []


def get(pattern: str):
    compiled = re.compile(f"^{pattern}$")
    def decorator(fn: Callable) -> Callable:
        _GET_ROUTES.append((compiled, fn))
        return fn
    return decorator


def post(path: str):
    def decorator(fn: Callable) -> Callable:
        _POST_ROUTES.append((path, fn))
        return fn
    return decorator


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.message = message


def dispatch_get(path: str, handler: SimpleHTTPRequestHandler) -> bool:
    path_only = path.split("?", 1)[0]
    for pattern, fn in _GET_ROUTES:
        m = pattern.match(path_only)
        if m:
            try:
                fn(handler, **m.groupdict())
            except ApiError:
                raise
            except Exception as exc:
                send_json(handler, 500, {"ok": False, "error": str(exc)})
            return True
    return False


def dispatch_post(path: str, handler: SimpleHTTPRequestHandler, payload: dict) -> bool:
    for route_path, fn in _POST_ROUTES:
        if path == route_path:
            try:
                fn(handler, payload)
            except ApiError:
                raise
            except Exception as exc:
                send_json(handler, 500, {"ok": False, "error": str(exc)})
            return True
    return False


def read_json(handler: SimpleHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    if length > 32_768:
        raise ApiError(413, "Request body too large")
    raw = handler.rfile.read(length)
    if not raw:
        return {}
    try:
        value = json.loads(raw.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise ApiError(400, "Invalid JSON") from exc
    if not isinstance(value, dict):
        raise ApiError(400, "JSON body must be an object")
    return value


def send_json(handler: SimpleHTTPRequestHandler, status: int, data: dict) -> None:
    encoded = json.dumps(data, indent=2).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(encoded)))
    handler.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
    handler.end_headers()
    handler.wfile.write(encoded)
