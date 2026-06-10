"""Automatic sighting memory: the system keeps the ledger, policies query it."""
import time

import roborun.sightings as sg
from roborun.behaviors import Robot


def setup_function(_):
    sg.reset()


def test_episode_counting(monkeypatch):
    t = [1000.0]
    monkeypatch.setattr(time, "time", lambda: t[0])
    # continuous sighting = one episode, regardless of frame count
    for _ in range(20):
        sg.observe([{"label": "red door", "confidence": 0.9}],
                   pose={"x": 1, "z": 2}, source="arena")
        t[0] += 0.1
    assert sg.summary("red door")[0]["count"] == 1
    # gone for > EPISODE_GAP, seen again = second episode
    t[0] += 5.0
    sg.observe([{"label": "red door", "confidence": 0.9}],
               pose={"x": 5, "z": 6}, source="arena")
    row = sg.summary("red door")[0]
    assert row["count"] == 2
    assert row["poses"] == [{"x": 1, "z": 2}, {"x": 5, "z": 6}]


def test_summary_filters_and_sorts():
    sg.observe([{"label": "blue door"}], source="arena")
    time.sleep(0)
    for _ in range(2):
        sg.observe([{"label": "person"}], source="camera")
    assert [r["label"] for r in sg.summary()] == ["blue door", "person"]
    assert sg.summary("person")[0]["source"] == "camera"
    assert sg.summary("cat") == []


def test_robot_seen_queries_the_system():
    sg.observe([{"label": "red door"}], pose={"x": 0, "z": 0}, source="arena")
    rows = Robot("t").seen("red door")
    assert rows and rows[0]["count"] == 1


def test_reset_clears():
    sg.observe([{"label": "x"}])
    sg.reset()
    assert sg.summary() == []
