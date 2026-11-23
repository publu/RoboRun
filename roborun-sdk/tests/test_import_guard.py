"""Hard rule: no torch, transformers or cv2 in roborun-rt. Enforced in a fresh interpreter (this pytest
process may hold cv2 from other suites), plus a static scan of the rt/connector sources."""
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from roborun_sdk.cli import RT_MODULES

SRC = Path(__file__).resolve().parents[1] / "src"


def _run(code: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-I", "-c", f"import sys; sys.path.insert(0, {str(SRC)!r})\n" + code],
                          capture_output=True, text=True, timeout=120)


def test_rt_modules_load_no_forbidden_module():
    proc = _run("import importlib, json\n"
                "from roborun_sdk.rt import guard\n"
                "guard.install()\n"
                f"[importlib.import_module(m) for m in {RT_MODULES!r}]\n"
                "print(json.dumps(guard.loaded()))")
    assert proc.returncode == 0, proc.stderr
    assert json.loads(proc.stdout.strip().splitlines()[-1]) == []


def test_guard_blocks_cv2_torch_transformers_after_install():
    proc = _run("from roborun_sdk.rt import guard\n"
                "guard.install()\n"
                "out = []\n"
                "for name in ('cv2', 'torch', 'transformers'):\n"
                "    try:\n"
                "        __import__(name)\n"
                "        out.append(name + ':imported')\n"
                "    except guard.ForbiddenImport:\n"
                "        out.append(name + ':blocked')\n"
                "print(','.join(out))")
    assert proc.returncode == 0, proc.stderr
    assert proc.stdout.strip().splitlines()[-1] == "cv2:blocked,torch:blocked,transformers:blocked"


def test_guard_refuses_to_start_when_already_loaded():
    proc = _run("import json\n"
                "sys.modules['cv2'] = type(sys)('cv2')\n"
                "from roborun_sdk.rt import guard\n"
                "try:\n"
                "    guard.install(); print('installed')\n"
                "except RuntimeError as e:\n"
                "    print('refused')")
    assert proc.stdout.strip().splitlines()[-1] == "refused"


def test_static_scan_of_rt_and_connectors():
    pattern = re.compile(r"^\s*(import|from)\s+(torch|transformers|cv2)\b", re.M)
    offenders = []
    for path in list((SRC / "roborun_sdk/rt").rglob("*.py")) + list((SRC / "roborun_sdk/connectors").rglob("*.py")):
        if pattern.search(path.read_text()):
            offenders.append(str(path.relative_to(SRC)))
    assert offenders == []


# The SDK runs on the computer wired to the robot: it imports no heavy ML package.
HEAVY_ML = ("torch", "torchvision", "transformers", "tensorflow", "jax", "diffusers", "onnxruntime")


def _sdk_text() -> str:
    return "\n".join(p.read_text() for p in (SRC / "roborun_sdk").rglob("*.py"))


def test_no_heavy_ml_imports_in_the_sdk():
    pattern = re.compile(r"^\s*(import|from)\s+(" + "|".join(HEAVY_ML) + r")\b", re.M)
    assert not pattern.search(_sdk_text())


def test_no_service_only_names_in_the_sdk():
    """Maintainers can list more names the SDK must never contain (module, key or setting names) in
    ROBORUN_SDK_FORBIDDEN_NAMES, comma-separated; none may appear anywhere in the SDK source. Unset: nothing to check."""
    names = [n.strip() for n in os.environ.get("ROBORUN_SDK_FORBIDDEN_NAMES", "").split(",") if n.strip()]
    text = _sdk_text()
    assert [n for n in names if n in text] == []
