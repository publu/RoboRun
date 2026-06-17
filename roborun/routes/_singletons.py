"""Lazy singletons shared across route modules."""
from __future__ import annotations

import os
from pathlib import Path

_webcam = None
_simulator = None
_spatial_memory = None
_agent = None


def get_webcam():
    global _webcam
    if _webcam is None:
        from roborun.webcam import WebcamPipeline
        _webcam = WebcamPipeline()
    return _webcam


def get_simulator():
    global _simulator
    if _simulator is None:
        from roborun.simulator import SimulatorRunner
        _simulator = SimulatorRunner()
    return _simulator


_spatial_memory_key = None


def get_memory():
    """The search index for the active project/environment. Rebuilt when the
    active context switches so each project searches only its own data."""
    global _spatial_memory, _spatial_memory_key
    key = None
    try:
        from roborun import projects
        a = projects.active()
        if a:
            key = (a["project"], a["environment"])
    except Exception:
        pass
    if _spatial_memory is None or _spatial_memory_key != key:
        from roborun.spatial_memory import SpatialMemoryStore
        _spatial_memory = SpatialMemoryStore(
            s3_bucket=os.environ.get("ROBORUN_S3_BUCKET"),
            s3_prefix=os.environ.get("ROBORUN_S3_PREFIX", "roborun/memories/"),
            s3_endpoint=os.environ.get("ROBORUN_S3_ENDPOINT"),
        )
        _spatial_memory_key = key
    return _spatial_memory


def get_scene_builder():
    from roborun.scene_builder import SceneBuilder
    return SceneBuilder.get()


def get_agent():
    global _agent
    if _agent is None:
        if not os.environ.get("ANTHROPIC_API_KEY"):
            return "unavailable"
        try:
            from roborun.agent import FastRobotAgent
            _agent = FastRobotAgent()
        except Exception:
            _agent = "unavailable"
    return _agent
