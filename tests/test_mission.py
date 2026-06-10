"""Mission compiler: words in, validated policy out (LLM stubbed)."""
import roborun.llm as llm
from roborun.mission import compile_mission, _extract_code

GOOD = '''```python
from roborun.behaviors import behavior

@behavior(hz=10)
def player_policy(robot):
    robot.move(forward=0.5)
```'''


def test_compiles_code_block(monkeypatch):
    monkeypatch.setattr(llm, "complete", lambda *a, **k: GOOD)
    r = compile_mission("drive forward")
    assert r["ok"]
    assert r["source"].startswith("# mission: drive forward")
    assert "@behavior" in r["source"]
    compile(r["source"], "x.py", "exec")  # generated artifact is valid python


def test_rejects_prose(monkeypatch):
    monkeypatch.setattr(llm, "complete", lambda *a, **k: "Sure! First you should...")
    assert not compile_mission("x")["ok"]


def test_rejects_broken_python(monkeypatch):
    monkeypatch.setattr(llm, "complete",
                        lambda *a, **k: "```python\n@behavior(hz=10)\ndef player_policy(robot:\n```")
    r = compile_mission("x")
    assert not r["ok"] and "syntax" in r["error"]


def test_rejects_wrong_function(monkeypatch):
    monkeypatch.setattr(llm, "complete",
                        lambda *a, **k: "```python\nfrom roborun.behaviors import behavior\n@behavior(hz=10)\ndef other(robot):\n    pass\n```")
    assert not compile_mission("x")["ok"]


def test_llm_down_is_clean_error(monkeypatch):
    monkeypatch.setattr(llm, "complete",
                        lambda *a, **k: (_ for _ in ()).throw(RuntimeError("no key")))
    r = compile_mission("x")
    assert not r["ok"] and "LLM unavailable" in r["error"]


def test_extract_handles_bare_code():
    assert "def " in _extract_code("def player_policy(robot):\n    pass")
