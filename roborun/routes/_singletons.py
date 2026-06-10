"""Lazy singletons shared across route modules."""
from __future__ import annotations

import os
from pathlib import Path

_webcam = None
_dataset = None
_simulator = None
_spatial_memory = None
_agent = None


def get_webcam():
    global _webcam
    if _webcam is None:
        from roborun.webcam import WebcamPipeline
        _webcam = WebcamPipeline()
    return _webcam


def get_dataset():
    global _dataset
    if _dataset is None:
        from roborun.dataset import DatasetCollector
        _dataset = DatasetCollector()
    return _dataset


def get_simulator():
    global _simulator
    if _simulator is None:
        from roborun.simulator import SimulatorRunner
        _simulator = SimulatorRunner()
    return _simulator


def get_memory():
    global _spatial_memory
    if _spatial_memory is None:
        from roborun.spatial_memory import SpatialMemoryStore
        _spatial_memory = SpatialMemoryStore(
            s3_bucket=os.environ.get("ROBORUN_S3_BUCKET"),
            s3_prefix=os.environ.get("ROBORUN_S3_PREFIX", "roborun/memories/"),
            s3_endpoint=os.environ.get("ROBORUN_S3_ENDPOINT"),
        )
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
