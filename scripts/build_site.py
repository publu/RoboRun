#!/usr/bin/env python3
"""Assemble the static arena site (GitHub Pages / any static host).

site/
  index.html, arena.html, arena.js, wasm.js, …   (roborun/web/, verbatim)
  py/shim.py                                      (browser runtime glue)
  py/roborun/*.py                                 (the real modules, for Pyodide)

The page detects at load time whether a local roborun server answers; on a
static host it boots these python modules in the browser instead. Test
locally with:  python scripts/build_site.py && python -m http.server -d site
"""
from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "site"

# everything the browser runtime imports (directly or transitively)
PY_MODULES = ["__init__.py", "arena.py", "behaviors.py", "sightings.py",
              "integrity.py", "recorder.py", "anchor.py", "events.py",
              "llm.py"]


def main() -> None:
    vercel_link = None
    if (SITE / ".vercel").exists():        # survive rebuilds: keep the deploy link
        vercel_link = (SITE / ".vercel").rename(ROOT / ".vercel-link-tmp")
    if SITE.exists():
        shutil.rmtree(SITE)
    shutil.copytree(ROOT / "roborun" / "web", SITE)
    if vercel_link is not None:
        vercel_link.rename(SITE / ".vercel")
    dst = SITE / "py" / "roborun"
    dst.mkdir(parents=True, exist_ok=True)
    for name in PY_MODULES:
        shutil.copy(ROOT / "roborun" / name, dst / name)
    # dashboard everywhere: the Home dashboard is the landing; the playable
    # in-browser cockpit stays reachable at /sim (arena.html, Pyodide).
    shutil.copy(SITE / "home.html", SITE / "index.html")
    # /sim as a real file, not a vercel.json rewrite: Vercel drops rewrites under
    # a Root Directory, so `/sim?level=…` 404'd on the hosted site. A physical
    # sim.html (copy of arena.html) resolves directly; the ?level query rides
    # through client-side. The local server still maps /sim → arena.html itself.
    shutil.copy(SITE / "arena.html", SITE / "sim.html")
    n = sum(1 for _ in SITE.rglob("*") if _.is_file())
    print(f"site/ assembled — {n} files")


if __name__ == "__main__":
    main()
