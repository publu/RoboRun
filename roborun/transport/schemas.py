"""Bundled ROS 2 message schemas for the common families (spec §1.2.1).

Two layers:

  * `message_fields(type)` — field definitions as plain dicts, for agents
    (get_message_details) and the rosbridge backend (JSON encoding).
  * `dds_types()` — CycloneDDS IdlStruct classes for the same families, built
    lazily so `cyclonedds` stays optional. This is what lets the DDS backend
    publish and (crucially, for recording) *deserialize* general messages
    instead of hardcoding Twist.

Families: std_msgs, geometry_msgs, sensor_msgs, nav_msgs, tf2_msgs.
Robots advertising other types still work over rosbridge/native; over DDS
they surface in the capability matrix as not-deserializable rather than
failing silently. XTypes/TypeObject discovery is the documented next step.
"""
from __future__ import annotations

from typing import Any

# ── JSON-side field definitions ───────────────────────────────────────────

_HEADER = {"stamp": "builtin_interfaces/Time {sec: int32, nanosec: uint32}",
           "frame_id": "string"}

MESSAGE_FIELDS: dict[str, dict[str, Any]] = {
    "std_msgs/String": {"data": "string"},
    "std_msgs/Header": _HEADER,
    "geometry_msgs/Vector3": {"x": "float64", "y": "float64", "z": "float64"},
    "geometry_msgs/Point": {"x": "float64", "y": "float64", "z": "float64"},
    "geometry_msgs/Quaternion": {"x": "float64", "y": "float64", "z": "float64", "w": "float64"},
    "geometry_msgs/Pose": {"position": "geometry_msgs/Point", "orientation": "geometry_msgs/Quaternion"},
    "geometry_msgs/PoseStamped": {"header": "std_msgs/Header", "pose": "geometry_msgs/Pose"},
    "geometry_msgs/Twist": {"linear": "geometry_msgs/Vector3", "angular": "geometry_msgs/Vector3"},
    "geometry_msgs/TwistStamped": {"header": "std_msgs/Header", "twist": "geometry_msgs/Twist"},
    "geometry_msgs/Transform": {"translation": "geometry_msgs/Vector3", "rotation": "geometry_msgs/Quaternion"},
    "geometry_msgs/TransformStamped": {"header": "std_msgs/Header", "child_frame_id": "string",
                                       "transform": "geometry_msgs/Transform"},
    "geometry_msgs/PoseWithCovariance": {"pose": "geometry_msgs/Pose", "covariance": "float64[36]"},
    "geometry_msgs/TwistWithCovariance": {"twist": "geometry_msgs/Twist", "covariance": "float64[36]"},
    "nav_msgs/Odometry": {"header": "std_msgs/Header", "child_frame_id": "string",
                          "pose": "geometry_msgs/PoseWithCovariance",
                          "twist": "geometry_msgs/TwistWithCovariance"},
    "sensor_msgs/CompressedImage": {"header": "std_msgs/Header", "format": "string",
                                    "data": "uint8[]"},
    "sensor_msgs/Imu": {"header": "std_msgs/Header",
                        "orientation": "geometry_msgs/Quaternion",
                        "orientation_covariance": "float64[9]",
                        "angular_velocity": "geometry_msgs/Vector3",
                        "angular_velocity_covariance": "float64[9]",
                        "linear_acceleration": "geometry_msgs/Vector3",
                        "linear_acceleration_covariance": "float64[9]"},
    "sensor_msgs/JointState": {"header": "std_msgs/Header", "name": "string[]",
                               "position": "float64[]", "velocity": "float64[]",
                               "effort": "float64[]"},
    "sensor_msgs/LaserScan": {"header": "std_msgs/Header",
                              "angle_min": "float32", "angle_max": "float32",
                              "angle_increment": "float32", "time_increment": "float32",
                              "scan_time": "float32", "range_min": "float32",
                              "range_max": "float32", "ranges": "float32[]",
                              "intensities": "float32[]"},
    "sensor_msgs/BatteryState": {"header": "std_msgs/Header", "voltage": "float32",
                                 "temperature": "float32", "current": "float32",
                                 "charge": "float32", "capacity": "float32",
                                 "design_capacity": "float32", "percentage": "float32",
                                 "power_supply_status": "uint8", "power_supply_health": "uint8",
                                 "power_supply_technology": "uint8", "present": "bool",
                                 "cell_voltage": "float32[]", "cell_temperature": "float32[]",
                                 "location": "string", "serial_number": "string"},
    "tf2_msgs/TFMessage": {"transforms": "geometry_msgs/TransformStamped[]"},
}

SUPPORTED_TYPES = tuple(sorted(MESSAGE_FIELDS))


def normalize_type(msg_type: str) -> str:
    """'geometry_msgs/msg/Twist' or 'geometry_msgs::msg::dds_::Twist_' → 'geometry_msgs/Twist'."""
    t = msg_type.strip()
    if "::" in t:
        parts = [p for p in t.split("::") if p not in ("msg", "dds_")]
        t = "/".join(parts).rstrip("_")
    parts = t.split("/")
    if len(parts) == 3 and parts[1] == "msg":
        t = f"{parts[0]}/{parts[2]}"
    return t


def dds_typename(msg_type: str) -> str:
    """'geometry_msgs/Twist' → 'geometry_msgs::msg::dds_::Twist_' (RMW mangling)."""
    pkg, name = normalize_type(msg_type).split("/", 1)
    return f"{pkg}::msg::dds_::{name}_"


def message_fields(msg_type: str) -> dict | None:
    return MESSAGE_FIELDS.get(normalize_type(msg_type))


# ── DDS-side IdlStruct registry (lazy: cyclonedds optional) ──────────────

_dds_cache: dict[str, Any] | None = None


def dds_types() -> dict[str, Any]:
    """{ros2 type → IdlStruct class} for the bundled families.

    Raises ImportError when cyclonedds is missing. Field order matches the
    .msg definitions exactly — CDR is positional.
    """
    global _dds_cache
    if _dds_cache is not None:
        return _dds_cache

    from dataclasses import dataclass, field
    from cyclonedds.idl import IdlStruct
    from cyclonedds.idl.types import array, sequence, uint8, uint32, int32, float32, float64

    def _zeros(n: int):
        return field(default_factory=lambda: [0.0] * n)

    @dataclass
    class Time(IdlStruct, typename="builtin_interfaces::msg::dds_::Time_"):
        sec: int32 = 0
        nanosec: uint32 = 0

    @dataclass
    class Header(IdlStruct, typename="std_msgs::msg::dds_::Header_"):
        stamp: Time = field(default_factory=Time)
        frame_id: str = ""

    @dataclass
    class String(IdlStruct, typename="std_msgs::msg::dds_::String_"):
        data: str = ""

    @dataclass
    class Vector3(IdlStruct, typename="geometry_msgs::msg::dds_::Vector3_"):
        x: float64 = 0.0
        y: float64 = 0.0
        z: float64 = 0.0

    @dataclass
    class Point(IdlStruct, typename="geometry_msgs::msg::dds_::Point_"):
        x: float64 = 0.0
        y: float64 = 0.0
        z: float64 = 0.0

    @dataclass
    class Quaternion(IdlStruct, typename="geometry_msgs::msg::dds_::Quaternion_"):
        x: float64 = 0.0
        y: float64 = 0.0
        z: float64 = 0.0
        w: float64 = 1.0

    @dataclass
    class Pose(IdlStruct, typename="geometry_msgs::msg::dds_::Pose_"):
        position: Point = field(default_factory=Point)
        orientation: Quaternion = field(default_factory=Quaternion)

    @dataclass
    class PoseStamped(IdlStruct, typename="geometry_msgs::msg::dds_::PoseStamped_"):
        header: Header = field(default_factory=Header)
        pose: Pose = field(default_factory=Pose)

    @dataclass
    class Twist(IdlStruct, typename="geometry_msgs::msg::dds_::Twist_"):
        linear: Vector3 = field(default_factory=Vector3)
        angular: Vector3 = field(default_factory=Vector3)

    @dataclass
    class TwistStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TwistStamped_"):
        header: Header = field(default_factory=Header)
        twist: Twist = field(default_factory=Twist)

    @dataclass
    class Transform(IdlStruct, typename="geometry_msgs::msg::dds_::Transform_"):
        translation: Vector3 = field(default_factory=Vector3)
        rotation: Quaternion = field(default_factory=Quaternion)

    @dataclass
    class TransformStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TransformStamped_"):
        header: Header = field(default_factory=Header)
        child_frame_id: str = ""
        transform: Transform = field(default_factory=Transform)

    @dataclass
    class PoseWithCovariance(IdlStruct, typename="geometry_msgs::msg::dds_::PoseWithCovariance_"):
        pose: Pose = field(default_factory=Pose)
        covariance: array[float64, 36] = _zeros(36)

    @dataclass
    class TwistWithCovariance(IdlStruct, typename="geometry_msgs::msg::dds_::TwistWithCovariance_"):
        twist: Twist = field(default_factory=Twist)
        covariance: array[float64, 36] = _zeros(36)

    @dataclass
    class Odometry(IdlStruct, typename="nav_msgs::msg::dds_::Odometry_"):
        header: Header = field(default_factory=Header)
        child_frame_id: str = ""
        pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
        twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)

    @dataclass
    class CompressedImage(IdlStruct, typename="sensor_msgs::msg::dds_::CompressedImage_"):
        header: Header = field(default_factory=Header)
        format: str = ""
        data: sequence[uint8] = field(default_factory=list)

    @dataclass
    class Imu(IdlStruct, typename="sensor_msgs::msg::dds_::Imu_"):
        header: Header = field(default_factory=Header)
        orientation: Quaternion = field(default_factory=Quaternion)
        orientation_covariance: array[float64, 9] = _zeros(9)
        angular_velocity: Vector3 = field(default_factory=Vector3)
        angular_velocity_covariance: array[float64, 9] = _zeros(9)
        linear_acceleration: Vector3 = field(default_factory=Vector3)
        linear_acceleration_covariance: array[float64, 9] = _zeros(9)

    @dataclass
    class JointState(IdlStruct, typename="sensor_msgs::msg::dds_::JointState_"):
        header: Header = field(default_factory=Header)
        name: sequence[str] = field(default_factory=list)
        position: sequence[float64] = field(default_factory=list)
        velocity: sequence[float64] = field(default_factory=list)
        effort: sequence[float64] = field(default_factory=list)

    @dataclass
    class LaserScan(IdlStruct, typename="sensor_msgs::msg::dds_::LaserScan_"):
        header: Header = field(default_factory=Header)
        angle_min: float32 = 0.0
        angle_max: float32 = 0.0
        angle_increment: float32 = 0.0
        time_increment: float32 = 0.0
        scan_time: float32 = 0.0
        range_min: float32 = 0.0
        range_max: float32 = 0.0
        ranges: sequence[float32] = field(default_factory=list)
        intensities: sequence[float32] = field(default_factory=list)

    @dataclass
    class BatteryState(IdlStruct, typename="sensor_msgs::msg::dds_::BatteryState_"):
        header: Header = field(default_factory=Header)
        voltage: float32 = 0.0
        temperature: float32 = 0.0
        current: float32 = 0.0
        charge: float32 = 0.0
        capacity: float32 = 0.0
        design_capacity: float32 = 0.0
        percentage: float32 = 0.0
        power_supply_status: uint8 = 0
        power_supply_health: uint8 = 0
        power_supply_technology: uint8 = 0
        present: bool = False
        cell_voltage: sequence[float32] = field(default_factory=list)
        cell_temperature: sequence[float32] = field(default_factory=list)
        location: str = ""
        serial_number: str = ""

    @dataclass
    class TFMessage(IdlStruct, typename="tf2_msgs::msg::dds_::TFMessage_"):
        transforms: sequence[TransformStamped] = field(default_factory=list)

    _dds_cache = {
        "std_msgs/String": String,
        "std_msgs/Header": Header,
        "geometry_msgs/Vector3": Vector3,
        "geometry_msgs/Point": Point,
        "geometry_msgs/Quaternion": Quaternion,
        "geometry_msgs/Pose": Pose,
        "geometry_msgs/PoseStamped": PoseStamped,
        "geometry_msgs/Twist": Twist,
        "geometry_msgs/TwistStamped": TwistStamped,
        "geometry_msgs/Transform": Transform,
        "geometry_msgs/TransformStamped": TransformStamped,
        "geometry_msgs/PoseWithCovariance": PoseWithCovariance,
        "geometry_msgs/TwistWithCovariance": TwistWithCovariance,
        "nav_msgs/Odometry": Odometry,
        "sensor_msgs/CompressedImage": CompressedImage,
        "sensor_msgs/Imu": Imu,
        "sensor_msgs/JointState": JointState,
        "sensor_msgs/LaserScan": LaserScan,
        "sensor_msgs/BatteryState": BatteryState,
        "tf2_msgs/TFMessage": TFMessage,
    }
    return _dds_cache


def dds_class_for(msg_type: str):
    """IdlStruct class for a ROS 2/DDS type name, or None if not bundled."""
    try:
        types = dds_types()
    except ImportError:
        return None
    return types.get(normalize_type(msg_type))


def to_dict(msg) -> dict:
    """IdlStruct → plain dict (bytes-safe for JSON paths)."""
    import dataclasses
    if dataclasses.is_dataclass(msg):
        return {f.name: to_dict(getattr(msg, f.name)) for f in dataclasses.fields(msg)}
    if isinstance(msg, (list, tuple)):
        return [to_dict(m) for m in msg]  # type: ignore[return-value]
    if isinstance(msg, (bytes, bytearray)):
        return list(msg)  # type: ignore[return-value]
    return msg


def from_dict(cls, data: dict):
    """Plain dict → IdlStruct instance, missing fields defaulted."""
    import dataclasses
    import typing
    kwargs = {}
    hints = typing.get_type_hints(cls)
    for f in dataclasses.fields(cls):
        if f.name not in data:
            continue
        val = data[f.name]
        ftype = hints.get(f.name, f.type)
        if dataclasses.is_dataclass(ftype) and isinstance(val, dict):
            kwargs[f.name] = from_dict(ftype, val)
        elif isinstance(val, list):
            inner = typing.get_args(ftype)
            if inner and dataclasses.is_dataclass(inner[0]):
                kwargs[f.name] = [from_dict(inner[0], v) if isinstance(v, dict) else v
                                  for v in val]
            else:
                kwargs[f.name] = val
        else:
            kwargs[f.name] = val
    return cls(**kwargs)
