"""End-to-end: the whole system as ONE loop, no hardware.

sim/synthetic source → YOLO+CLIP(fake) → sealed MCAP → live index → search over time
→ scenario score → analytics → curate a dataset. Proves the pieces compose into the
single system the user described (sim/robot/production, track+search over time).
"""
from __future__ import annotations

import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")


@pytest.fixture()
def env(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBORUN_STATE_DIR", str(tmp_path))
    from roborun.routes import _singletons
    _singletons._spatial_memory = None
    return tmp_path


def _fake_embed(frame):
    return frame.reshape(-1, 3).mean(axis=0).astype(np.float32)


def test_full_loop(env):
    from roborun.synthetic_camera import SyntheticCamera
    from roborun.session import PerceptionSession, search, export_dataset
    from roborun.routes._singletons import get_memory
    from roborun.recorder import RunRecorder
    from roborun.observations import StreamingExtractor
    from roborun.events import runs_root

    store = get_memory()

    # 1) RECORD + INDEX: a synthetic "production" camera through the unified session,
    #    writing to a sealed MCAP that streams into the searchable index.
    rec = RunRecorder(robot_id="dock-bot", root=runs_root(), checkpoint_interval=0.05)
    rec.extractor = StreamingExtractor(store, robot_id="dock-bot", run_id=rec.run_id)
    cam = SyntheticCamera(label="person")
    sess = PerceptionSession(cam, store, mode="production", source_id="dock",
                             recorder=rec, embed_fn=_fake_embed, hz=30)
    cam.start()
    try:
        for _ in range(8):
            sess.tick()
            time.sleep(0.02)
    finally:
        cam.stop()
    seal = rec.close(do_anchor=False)

    # the run is sealed + verifiable, and carries camera + detections + clip
    assert seal["merkle_root"]
    assert seal["message_counts"].get("/camera/dock")
    assert seal["message_counts"].get("/clip/embeddings")

    # 2) SEARCH OVER TIME across all history, by label and semantic
    hits = search(store, "person", by="label")
    assert hits and hits[0]["source_id"] == "dock"
    assert hits[0]["source"] == "production"  # live index tagged with the real mode
    recent = search(store, "person", by="label", since=time.time() - 3600)
    assert len(recent) == len(hits)

    # 3) SCORE a scenario (the Define/Analyze surface) and see it aggregate
    from roborun import scenario as S
    with S.scenario("dock_patrol", suite="ops", tags=["nav"], seed=1) as run:
        run.metric("sightings", len(hits))
        run.evaluate("coverage", seen=1.0)
        run.passed()
    assert S.list_suites()[0]["pass_rate"] == 1.0

    # 4) ANALYTICS reflects everything tracked
    assert store.label_histogram()[0]["label"] == "person"
    robots = {r["robot_id"] for r in store.robots_breakdown()}
    assert "dock-bot" in robots

    # 5) CURATE a dataset from the search — sealed provenance
    out = env / "dataset"
    ds = export_dataset(store, "person", str(out), by="label")
    assert ds["count"] >= 1 and (out / "labels.jsonl").exists()
