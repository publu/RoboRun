"""ROS 1 + ROS 2 support: rosbridge speaks both; we detect which via heuristics."""
from __future__ import annotations

from roborun.rosbridge import RosbridgeClient
from roborun.transport import CAPABILITY_MATRIX


def _client_with_topics(topics):
    c = RosbridgeClient.__new__(RosbridgeClient)        # no socket
    c.list_topics = lambda timeout=5.0: topics
    return c


def test_detects_ros2():
    c = _client_with_topics([
        {"topic": "/parameter_events", "type": "rcl_interfaces/msg/ParameterEvent"},
        {"topic": "/scan", "type": "sensor_msgs/msg/LaserScan"},
        {"topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"},
    ])
    assert c.ros_version() == "ros2"


def test_detects_ros1():
    c = _client_with_topics([
        {"topic": "/rosout_agg", "type": "rosgraph_msgs/Log"},
        {"topic": "/scan", "type": "sensor_msgs/LaserScan"},
        {"topic": "/cmd_vel", "type": "geometry_msgs/Twist"},
    ])
    assert c.ros_version() == "ros1"


def test_unknown_when_ambiguous():
    c = _client_with_topics([{"topic": "/scan", "type": "sensor_msgs/LaserScan"}])
    assert c.ros_version() == "unknown"


def test_both_versions_share_the_rosbridge_transport():
    # the value prop: one transport, both ROS versions (DDS is ROS2-only)
    assert CAPABILITY_MATRIX["rosbridge"]["subscribe"] is True
    assert CAPABILITY_MATRIX["rosbridge"]["services"] is True
    # standard topics used by ros_telemetry exist in both ROS1 and ROS2
    from roborun.ros_telemetry import STANDARD_TOPICS
    names = {t[0] if isinstance(t, (tuple, list)) else t for t in STANDARD_TOPICS}
    assert "/odom" in names and "/scan" in names
