"""Transport tests — schemas, capability matrix, tap-to-recorder routing."""
import time

import pytest

from roborun.transport import (
    CAPABILITY_MATRIX, Robot, Transport, message_fields,
)
from roborun.transport.schemas import (
    MESSAGE_FIELDS, dds_typename, normalize_type,
)
from roborun.transport.tap import Tap
from roborun.recorder import RunRecorder, verify_mcap_run


def test_normalize_type_forms():
    assert normalize_type("geometry_msgs/Twist") == "geometry_msgs/Twist"
    assert normalize_type("geometry_msgs/msg/Twist") == "geometry_msgs/Twist"
    assert normalize_type("geometry_msgs::msg::dds_::Twist_") == "geometry_msgs/Twist"


def test_dds_typename_mangling():
    assert dds_typename("nav_msgs/Odometry") == "nav_msgs::msg::dds_::Odometry_"
    assert dds_typename("sensor_msgs/msg/CompressedImage") == \
        "sensor_msgs::msg::dds_::CompressedImage_"


def test_bundled_families_present():
    families = {t.split("/")[0] for t in MESSAGE_FIELDS}
    assert {"std_msgs", "geometry_msgs", "sensor_msgs", "nav_msgs", "tf2_msgs"} <= families
    # not just Twist anymore
    assert len(MESSAGE_FIELDS) >= 15
    assert message_fields("nav_msgs/msg/Odometry")["child_frame_id"] == "string"


def test_capability_matrix_shape():
    for backend in ("dds", "rosbridge", "native"):
        caps = CAPABILITY_MATRIX[backend]
        assert {"subscribe", "publish", "services", "actions", "params"} <= set(caps)
    assert CAPABILITY_MATRIX["dds"]["services"] is False
    assert CAPABILITY_MATRIX["rosbridge"]["services"] is True


class FakeTransport(Transport):
    name = "dds"

    def __init__(self, graph):
        self._graph = graph
        self.callbacks = {}

    def topics(self):
        return dict(self._graph)

    def subscribe(self, topic, callback, msg_type=None):
        self.callbacks[topic] = callback
        return topic

    def unsubscribe(self, topic):
        self.callbacks.pop(topic, None)

    def publish(self, topic, msg_type, msg):
        return {"ok": True}


def test_unsupported_capability_is_honest():
    t = FakeTransport({})
    r = t.call_service("/set_bool")
    assert not r["ok"]
    assert "not supported over dds" in r["error"]
    assert "rosbridge" in r["hint"]


def test_dds_types_require_cyclonedds():
    cyclonedds = pytest.importorskip("cyclonedds")
    from roborun.transport.schemas import dds_types, to_dict, from_dict
    types = dds_types()
    Twist = types["geometry_msgs/Twist"]
    msg = from_dict(Twist, {"linear": {"x": 0.5}, "angular": {"z": -0.2}})
    d = to_dict(msg)
    assert d["linear"]["x"] == 0.5 and d["angular"]["z"] == -0.2


def test_tap_routes_to_recorder_channels(tmp_path):
    graph = {
        "/odom": "nav_msgs/Odometry",
        "/camera/color/compressed": "sensor_msgs/CompressedImage",
        "/joint_states": "sensor_msgs/JointState",
    }
    ft = FakeTransport(graph)
    rec = RunRecorder(robot_id="tap", root=tmp_path, checkpoint_interval=0.01)
    tap = Tap(ft, rec, topics=["/odom", "/camera/*"])
    status = tap.start()
    assert set(status["topics"]) == {"/odom", "/camera/color/compressed"}

    t0 = time.time()
    stamp = {"stamp": {"sec": int(t0), "nanosec": 0}, "frame_id": "map"}
    for i in range(5):
        ft.callbacks["/odom"]({"header": stamp, "child_frame_id": "base",
                               "pose": {"pose": {"position": {"x": i, "y": 0, "z": 0},
                                                 "orientation": {"w": 1}}},
                               "twist": {}})
        ft.callbacks["/camera/color/compressed"](
            {"header": stamp, "format": "jpeg", "data": [255, 216, i]})
    tap.stop()
    seal = rec.close(do_anchor=False)

    counts = seal["message_counts"]
    assert counts["/pose"] == 5            # odometry routed to the pose channel
    assert counts["/odom"] == 5            # and kept verbatim
    assert counts["/camera/camera_color_compressed"] == 5
    assert verify_mcap_run(rec.mcap_path)["state"] == "consistent_unanchored"
    assert tap.status()["messages"]["/odom"] == 5


def test_tap_full_graph_when_no_patterns(tmp_path):
    ft = FakeTransport({"/a": "std_msgs/String", "/b": "std_msgs/String"})
    rec = RunRecorder(robot_id="tap2", root=tmp_path)
    tap = Tap(ft, rec)
    assert set(tap.start()["topics"]) == {"/a", "/b"}
    rec.close(do_anchor=False)
