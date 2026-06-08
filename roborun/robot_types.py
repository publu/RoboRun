"""Robot type system — drives UI adaptation, telemetry channels, and available tools."""

from __future__ import annotations

from enum import Enum
from typing import Any


class RobotType(Enum):
    QUADRUPED = "quadruped"
    HUMANOID = "humanoid"
    DRONE = "drone"
    ARM = "arm"
    WEBCAM_ONLY = "webcam_only"


_PROFILES: dict[RobotType, dict[str, Any]] = {
    RobotType.DRONE: {
        "label": "Drone",
        "icon": "✈",
        "color": "#40a0e0",
        "telemetry_channels": [
            "battery", "altitude", "gps", "orientation", "velocity",
            "airspeed", "heading", "satellites",
        ],
        "ui_panels": [
            "camera", "telemetry", "trajectory3d", "pointcloud",
            "depth_heatmap", "drone_controls",
        ],
        "control_scheme": "waypoint",
        "skills": [
            "takeoff", "land", "goto_waypoint", "set_altitude",
            "follow_target", "move", "camera_snapshot", "yolo_detections",
            "memory_search", "get_telemetry", "get_trajectory",
        ],
        "ros_topics": {
            "odom": "/mavros/local_position/odom",
            "battery": "/mavros/battery",
            "imu": "/mavros/imu/data",
            "gps": "/mavros/global_position/global",
            "cmd_vel": "/mavros/setpoint_velocity/cmd_vel",
            "status": "/mavros/state",
            "depth": "/camera/depth/image_raw",
            "camera": "/camera/image_raw/compressed",
            "points": "/camera/depth/points",
        },
    },
    RobotType.QUADRUPED: {
        "label": "Quadruped",
        "icon": "◈",
        "color": "#00d47e",
        "telemetry_channels": [
            "battery", "joint_states", "orientation", "velocity",
            "contact_forces", "imu",
        ],
        "ui_panels": [
            "camera", "telemetry", "trajectory3d", "pointcloud",
            "depth_heatmap", "dpad_controls", "sport_controls",
        ],
        "control_scheme": "cmd_vel",
        "skills": [
            "move", "explore", "follow_person", "follow_object",
            "find", "navigate", "patrol", "dog_mode", "stand", "sit",
            "camera_snapshot", "yolo_detections", "memory_search",
            "get_telemetry", "get_trajectory", "execute_skill",
        ],
        "ros_topics": {
            "odom": "/odom",
            "battery": "/battery_state",
            "imu": "/imu/data",
            "joint_states": "/joint_states",
            "cmd_vel": "/cmd_vel",
            "depth": "/camera/depth/image_raw",
            "camera": "/camera/image_raw/compressed",
            "points": "/camera/depth/points",
        },
    },
    RobotType.HUMANOID: {
        "label": "Humanoid",
        "icon": "⬡",
        "color": "#d4a030",
        "telemetry_channels": [
            "battery", "joint_states", "orientation", "velocity",
            "contact_forces", "imu", "torso_orientation",
        ],
        "ui_panels": [
            "camera", "telemetry", "trajectory3d", "pointcloud",
            "depth_heatmap", "dpad_controls",
        ],
        "control_scheme": "cmd_vel",
        "skills": [
            "move", "explore", "follow_person", "camera_snapshot",
            "yolo_detections", "memory_search", "get_telemetry",
            "get_trajectory", "execute_skill",
        ],
        "ros_topics": {
            "odom": "/odom",
            "battery": "/battery_state",
            "imu": "/imu/data",
            "joint_states": "/joint_states",
            "cmd_vel": "/cmd_vel",
            "depth": "/camera/depth/image_raw",
            "camera": "/camera/image_raw/compressed",
            "points": "/camera/depth/points",
        },
    },
    RobotType.ARM: {
        "label": "Robot Arm",
        "icon": "⚙",
        "color": "#e07040",
        "telemetry_channels": ["joint_states", "end_effector"],
        "ui_panels": ["camera", "telemetry", "trajectory3d"],
        "control_scheme": "joint",
        "skills": [
            "camera_snapshot", "yolo_detections", "memory_search",
            "get_telemetry", "execute_skill",
        ],
        "ros_topics": {
            "joint_states": "/joint_states",
            "camera": "/camera/image_raw/compressed",
        },
    },
    RobotType.WEBCAM_ONLY: {
        "label": "Webcam Only",
        "icon": "◉",
        "color": "#a0a0a0",
        "telemetry_channels": ["fps", "detection_count"],
        "ui_panels": ["camera", "telemetry"],
        "control_scheme": None,
        "skills": [
            "camera_snapshot", "yolo_detections", "memory_search",
        ],
        "ros_topics": {},
    },
}


def get_profile(robot_type: RobotType) -> dict[str, Any]:
    return {"type": robot_type.value, **_PROFILES[robot_type]}


def detect_type(
    blueprint: str = "",
    sim_robot_type: str = "",
    ros_topics: list[str] | None = None,
) -> RobotType:
    slug = blueprint.lower()
    if "drone" in slug or "quadrotor" in slug or "mavlink" in slug:
        return RobotType.DRONE
    if "arm" in slug or "manipulator" in slug:
        return RobotType.ARM

    if sim_robot_type == "drone":
        return RobotType.DRONE
    if sim_robot_type == "quadruped":
        return RobotType.QUADRUPED
    if sim_robot_type == "humanoid":
        return RobotType.HUMANOID

    if ros_topics:
        topic_set = set(ros_topics)
        if any("/mavros" in t for t in topic_set):
            return RobotType.DRONE
        if "/joint_states" in topic_set:
            joint_count_hint = sum(1 for t in topic_set if "joint" in t)
            if joint_count_hint > 3:
                return RobotType.HUMANOID
            return RobotType.QUADRUPED

    if any(k in slug for k in ("go1", "go2", "a1", "b1", "b2", "spot")):
        return RobotType.QUADRUPED
    if any(k in slug for k in ("g1", "h1", "atlas", "humanoid")):
        return RobotType.HUMANOID

    return RobotType.WEBCAM_ONLY
