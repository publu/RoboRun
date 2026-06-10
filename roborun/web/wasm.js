/* In-browser roborun runtime: the actual Python modules under Pyodide.
   arena.js imports this only when no local server answers (GitHub Pages,
   file://). A run sealed here verifies with the same Merkle chain as a
   local one — `python -m roborun.recorder verify` on the downloads. */

const PYODIDE_URL = "https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.mjs";
const PY_FILES = ["__init__.py", "arena.py", "behaviors.py", "sightings.py",
                  "integrity.py", "recorder.py", "anchor.py", "events.py",
                  "llm.py"];

export async function loadWasmRuntime(onStatus) {
  const status = onStatus || (() => {});
  status("loading python runtime… (~12 MB, cached after first visit)");
  const { loadPyodide } = await import(PYODIDE_URL);
  const py = await loadPyodide();

  status("installing mcap + cryptography…");
  await py.loadPackage("micropip");
  try { await py.loadPackage("cryptography"); } catch {}  // Ed25519 seal signing
  // mcap is pure python; deps=False because its lz4 extra has no wasm
  // build — zstandard (the writer's default compression) comes from the
  // pyodide distribution
  await py.loadPackage("zstandard");
  await py.runPythonAsync('import micropip; await micropip.install("mcap", deps=False)');

  status("loading roborun modules…");
  const base = new URL("py/", import.meta.url);
  py.FS.mkdirTree("/app/roborun");
  for (const f of PY_FILES) {
    const src = await (await fetch(new URL(`roborun/${f}`, base))).text();
    py.FS.writeFile(`/app/roborun/${f}`, src);
  }
  py.FS.writeFile("/app/shim.py",
    await (await fetch(new URL("shim.py", base))).text());
  py.runPython('import sys; sys.path.insert(0, "/app"); import shim');
  const shim = py.pyimport("shim");
  const J = (s) => JSON.parse(s);

  return {
    loadPolicy: (src) => J(shim.load_policy(src)),
    stopPolicy: () => shim.stop_policy(),
    tick: (state) => J(shim.tick(JSON.stringify(state))),
    emitEvent: (type, title, detail) =>
      shim.emit_event(type, title, JSON.stringify(detail || {})),
    sightings: () => J(shim.sightings_summary()),
    recordStart: () => J(shim.record_start("arena")),
    recordStop: () => {
      const r = J(shim.record_stop());
      if (r.ok) {
        r.files = {
          mcap: py.FS.readFile(r.mcap_path),
          chain: py.FS.readFile(r.chain_path, { encoding: "utf8" }),
          seal: py.FS.readFile(r.seal_path, { encoding: "utf8" }),
        };
      }
      return r;
    },
    verify: (name, mcapBytes, chain, seal) => {
      py.FS.mkdirTree("/verify");
      py.FS.writeFile(`/verify/${name}.mcap`, mcapBytes);
      py.FS.writeFile(`/verify/${name}.chain.jsonl`, chain);
      py.FS.writeFile(`/verify/${name}.seal`, seal);
      return J(shim.verify(`/verify/${name}.mcap`));
    },
  };
}
