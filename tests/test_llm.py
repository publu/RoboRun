"""Provider-agnostic LLM layer: resolution, request building, parsing."""
from __future__ import annotations

import pytest

from roborun import llm


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for var in ("ROBORUN_LLM", "ROBORUN_MODEL_FAST", "ROBORUN_MODEL_SMART",
                "ANTHROPIC_API_KEY", "OPENAI_API_KEY", "GEMINI_API_KEY",
                "GOOGLE_API_KEY", "OPENAI_BASE_URL", "OLLAMA_HOST"):
        monkeypatch.delenv(var, raising=False)


def test_resolve_defaults_to_local_without_keys():
    assert llm.resolve("fast") == ("ollama", llm.DEFAULTS["ollama"]["fast"])


def test_resolve_picks_provider_from_key(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-x")
    assert llm.resolve("fast") == ("openai", "gpt-5-mini")
    assert llm.resolve("smart") == ("openai", "gpt-5")


def test_resolve_tier_env_and_explicit_spec(monkeypatch):
    monkeypatch.setenv("ROBORUN_MODEL_FAST", "ollama:llama3.2")
    monkeypatch.setenv("ROBORUN_MODEL_SMART", "anthropic:claude-opus-4-8")
    assert llm.resolve("fast") == ("ollama", "llama3.2")
    assert llm.resolve("smart") == ("anthropic", "claude-opus-4-8")
    # explicit spec overrides everything
    assert llm.resolve("gemini:gemini-2.5-pro") == ("gemini", "gemini-2.5-pro")
    # bare provider spec gets that provider's smart default
    assert llm.resolve("anthropic") == ("anthropic", "claude-opus-4-8")


def test_legacy_roborun_llm_env(monkeypatch):
    monkeypatch.setenv("ROBORUN_LLM", "gemini")
    assert llm.resolve("fast") == ("gemini", "gemini-2.5-flash")


def test_build_anthropic_with_image(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    url, headers, body = llm._build("anthropic", "claude-haiku-4-5",
                                    "what do you see", "be terse", b"\xff\xd8jpg", 300)
    assert url.endswith("/v1/messages")
    assert headers["x-api-key"] == "k"
    assert body["system"] == "be terse"
    blocks = body["messages"][0]["content"]
    assert blocks[0]["type"] == "image" and blocks[1]["text"] == "what do you see"


def test_build_openai_respects_base_url(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "k")
    monkeypatch.setenv("OPENAI_BASE_URL", "http://localhost:8000/v1")
    url, headers, body = llm._build("openai", "local-model", "hi", None, None, 100)
    assert url == "http://localhost:8000/v1/chat/completions"
    assert headers["Authorization"] == "Bearer k"
    assert body["messages"] == [{"role": "user", "content": "hi"}]


def test_build_gemini_and_ollama():
    url, _, body = llm._build("gemini", "gemini-2.5-flash", "hi", "sys", None, 100)
    assert "gemini-2.5-flash:generateContent" in url
    assert body["system_instruction"]["parts"][0]["text"] == "sys"

    url, _, body = llm._build("ollama", "llama3.2", "hi", None, b"img", 100)
    assert url.endswith("/api/chat") and body["stream"] is False
    assert body["messages"][0]["images"]


def test_parse_all_providers():
    assert llm._parse("anthropic", {"content": [{"type": "text", "text": "a"},
                                                {"type": "text", "text": "b"}]}) == "ab"
    assert llm._parse("openai", {"choices": [{"message": {"content": " x "}}]}) == "x"
    assert llm._parse("gemini", {"candidates": [{"content": {"parts": [
        {"text": "y"}]}}]}) == "y"
    assert llm._parse("ollama", {"message": {"content": "z\n"}}) == "z"


def test_capabilities_shape(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "k")
    caps = llm.capabilities()
    assert caps["fast"]["provider"] == "anthropic"
    assert caps["configured_keys"] == ["anthropic"]
