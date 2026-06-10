"""Provider-agnostic LLM layer with fast/smart tiers — no SDK dependencies.

RoboRun does not assume a vendor. Four providers ship in-tree, all over
plain HTTP, and any OpenAI-compatible endpoint (vLLM, LM Studio, Groq, a
gateway) plugs in via a base-URL override:

    anthropic   ANTHROPIC_API_KEY
    openai      OPENAI_API_KEY      (+ OPENAI_BASE_URL for compatibles)
    gemini      GEMINI_API_KEY or GOOGLE_API_KEY
    ollama      no key — local, OLLAMA_HOST (default 127.0.0.1:11434)

Tiers, not a model: callers ask for "fast" (frequent, cheap — narration,
supervisory checks, classification) or "smart" (reasoning — diagnosis,
heartbeat reviews, planning). This is the speed-layers principle applied to
model choice (docs/SPEED_LAYERS.md): L3 work defaults to fast, L4 to smart.

Configuration — a spec is "provider" or "provider:model":

    ROBORUN_MODEL_FAST=ollama:llama3.2
    ROBORUN_MODEL_SMART=anthropic:claude-opus-4-8

Unset, the provider is whichever API key is present (checked in the order
above — a key order, not a quality ranking), falling back to local Ollama,
and the provider's default fast/smart models apply. ROBORUN_LLM=<provider>
(the legacy knob) still forces the provider for both tiers.
"""
from __future__ import annotations

import base64
import json
import os
import urllib.request
from typing import Any

DEFAULTS: dict[str, dict[str, str]] = {
    "anthropic": {"fast": "claude-haiku-4-5", "smart": "claude-opus-4-8"},
    "openai": {"fast": "gpt-5-mini", "smart": "gpt-5"},
    "gemini": {"fast": "gemini-2.5-flash", "smart": "gemini-2.5-pro"},
    "ollama": {"fast": os.environ.get("OLLAMA_MODEL", "llama3.2"),
               "smart": os.environ.get("OLLAMA_MODEL_SMART",
                                       os.environ.get("OLLAMA_MODEL", "llama3.2"))},
}

_TIMEOUT = {"anthropic": 60, "openai": 60, "gemini": 60, "ollama": 120}


def _detect_provider() -> str:
    if os.environ.get("ROBORUN_LLM"):
        return os.environ["ROBORUN_LLM"]
    if os.environ.get("ANTHROPIC_API_KEY"):
        return "anthropic"
    if os.environ.get("OPENAI_API_KEY"):
        return "openai"
    if os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY"):
        return "gemini"
    return "ollama"


def resolve(tier: str = "fast") -> tuple[str, str]:
    """A tier or explicit spec → (provider, model).

    Accepts "fast" / "smart", "provider", or "provider:model". Tier specs
    read ROBORUN_MODEL_FAST / ROBORUN_MODEL_SMART first.
    """
    spec = tier
    if tier in ("fast", "smart"):
        spec = os.environ.get(f"ROBORUN_MODEL_{tier.upper()}") or _detect_provider()
    else:
        tier = "smart"  # explicit specs get the smart default when model omitted
    if ":" in spec:
        provider, model = spec.split(":", 1)
        return provider, model
    provider = spec
    return provider, DEFAULTS.get(provider, DEFAULTS["ollama"])[tier]


def capabilities() -> dict[str, Any]:
    """What the LLM layer resolves to right now — for the UI and MCP."""
    fast = resolve("fast")
    smart = resolve("smart")
    return {"fast": {"provider": fast[0], "model": fast[1]},
            "smart": {"provider": smart[0], "model": smart[1]},
            "configured_keys": [p for p, k in (
                ("anthropic", "ANTHROPIC_API_KEY"), ("openai", "OPENAI_API_KEY"),
                ("gemini", "GEMINI_API_KEY")) if os.environ.get(k)]}


# ── request builders (pure: easy to test, no network) ───────────────────

def _build(provider: str, model: str, prompt: str, system: str | None,
           image_jpeg: bytes | None, max_tokens: int) -> tuple[str, dict, dict]:
    """(url, headers, body) for one completion request."""
    b64 = base64.b64encode(image_jpeg).decode() if image_jpeg else None

    if provider == "anthropic":
        content: list[dict] = []
        if b64:
            content.append({"type": "image", "source": {
                "type": "base64", "media_type": "image/jpeg", "data": b64}})
        content.append({"type": "text", "text": prompt})
        body: dict[str, Any] = {"model": model, "max_tokens": max_tokens,
                                "messages": [{"role": "user", "content": content}]}
        if system:
            body["system"] = system
        return ("https://api.anthropic.com/v1/messages",
                {"x-api-key": os.environ.get("ANTHROPIC_API_KEY", ""),
                 "anthropic-version": "2023-06-01"}, body)

    if provider == "openai":
        base = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1").rstrip("/")
        msgs: list[dict] = []
        if system:
            msgs.append({"role": "system", "content": system})
        if b64:
            msgs.append({"role": "user", "content": [
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                {"type": "text", "text": prompt}]})
        else:
            msgs.append({"role": "user", "content": prompt})
        return (f"{base}/chat/completions",
                {"Authorization": f"Bearer {os.environ.get('OPENAI_API_KEY', '')}"},
                {"model": model, "messages": msgs, "max_completion_tokens": max_tokens})

    if provider == "gemini":
        key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY", "")
        parts: list[dict] = []
        if b64:
            parts.append({"inline_data": {"mime_type": "image/jpeg", "data": b64}})
        parts.append({"text": prompt})
        body = {"contents": [{"parts": parts}],
                "generationConfig": {"maxOutputTokens": max_tokens}}
        if system:
            body["system_instruction"] = {"parts": [{"text": system}]}
        return (f"https://generativelanguage.googleapis.com/v1beta/models/"
                f"{model}:generateContent", {"x-goog-api-key": key}, body)

    # ollama (and the default for unknown providers, so a typo fails loudly
    # at the connection rather than building a request for the wrong API)
    host = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
    if not host.startswith("http"):
        host = f"http://{host}"
    msg: dict[str, Any] = {"role": "user", "content": prompt}
    if b64:
        msg["images"] = [b64]
    msgs = ([{"role": "system", "content": system}] if system else []) + [msg]
    return (f"{host}/api/chat", {},
            {"model": model, "messages": msgs, "stream": False,
             "options": {"num_predict": max_tokens}})


def _parse(provider: str, data: dict) -> str:
    if provider == "anthropic":
        return "".join(b.get("text", "") for b in data.get("content", [])).strip()
    if provider == "openai":
        return (data.get("choices") or [{}])[0].get("message", {}) \
            .get("content", "").strip()
    if provider == "gemini":
        parts = (data.get("candidates") or [{}])[0].get("content", {}).get("parts", [])
        return "".join(p.get("text", "") for p in parts).strip()
    return data.get("message", {}).get("content", "").strip()


# ── the one entry point ──────────────────────────────────────────────────

def complete(prompt: str, system: str | None = None,
             image_jpeg: bytes | None = None, tier: str = "fast",
             max_tokens: int = 300) -> str:
    """One completion. `tier` is "fast", "smart", or an explicit
    "provider:model" spec. Raises on failure — callers own the fallback."""
    provider, model = resolve(tier)
    url, headers, body = _build(provider, model, prompt, system,
                                image_jpeg, max_tokens)
    req = urllib.request.Request(
        url, data=json.dumps(body).encode(),
        headers={**headers, "content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=_TIMEOUT.get(provider, 60)) as resp:
        return _parse(provider, json.loads(resp.read()))
