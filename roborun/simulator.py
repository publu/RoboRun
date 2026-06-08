"""Lightweight MuJoCo simulator for RoboRun.

Runs headless, renders camera frames to the same MJPEG path the webcam
pipeline uses. Uses trained ONNX locomotion policies from dimOS to make
robots actually walk. WASD sends velocity commands to the policy.
"""

from __future__ import annotations

import time
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Event, RLock, Thread
from typing import Any

import cv2
import mujoco
import numpy as np

FRAME_PATH = Path("/tmp/roborun_frame.jpg")
ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets"
ROBOTS_DIR = ASSETS_DIR / "robots"
POLICIES_DIR = ASSETS_DIR / "policies"

ROBOT_CATALOG = {
    "unitree_go1": {
        "name": "Unitree Go1",
        "type": "quadruped",
        "xml": "unitree_go1.xml",
        "meshdir": "unitree_go1/assets",
        "policy": "unitree_go1_policy.onnx",
        "controller": "go1",
    },
    "unitree_g1": {
        "name": "Unitree G1",
        "type": "humanoid",
        "xml": "unitree_g1.xml",
        "meshdir": "unitree_g1/assets",
        "policy": "unitree_g1_policy.onnx",
        "controller": "g1",
    },
    "generic_drone": {
        "name": "Quadrotor",
        "type": "drone",
        "xml": "generic_drone.xml",
        "meshdir": None,
        "policy": None,
        "controller": "drone",
    },
}

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _load_model(robot_id: str) -> tuple[mujoco.MjModel, mujoco.MjData, dict]:
    info = ROBOT_CATALOG[robot_id]
    xml_path = ROBOTS_DIR / info["xml"]

    if info.get("meshdir"):
        scene_xml = (ROBOTS_DIR / "scene_empty.xml").read_text()
        scene_root = ET.fromstring(scene_xml)
        scene_root.set("model", f"{robot_id}_scene")

        robot_xml = xml_path.read_text()
        robot_root = ET.fromstring(robot_xml)

        compiler = robot_root.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(robot_root, "compiler")
        compiler.set("meshdir", str(ROBOTS_DIR / info["meshdir"]))

        for child in robot_root:
            if child.tag == "compiler":
                continue
            scene_root.append(child)

        compiler_scene = scene_root.find("compiler")
        if compiler_scene is None:
            compiler_scene = ET.SubElement(scene_root, "compiler")
        compiler_scene.set("meshdir", str(ROBOTS_DIR / info["meshdir"]))

        xml_str = ET.tostring(scene_root, encoding="unicode")
        model = mujoco.MjModel.from_xml_string(xml_str)
    else:
        model = mujoco.MjModel.from_xml_path(str(xml_path))

    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data, info


class _Go1Policy:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, policy_path: str) -> None:
        import onnxruntime as ort
        self._policy = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        self._default_angles = np.array(model.keyframe("home").qpos[7:])
        self._last_action = np.zeros_like(self._default_angles, dtype=np.float32)
        self._action_scale = 0.5
        self._command = np.zeros(3, dtype=np.float32)

    def set_command(self, fwd: float, left: float, turn: float) -> None:
        self._command[:] = [fwd, left, turn]

    def step(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        linvel = data.sensor("local_linvel").data
        gyro = data.sensor("gyro").data
        imu_xmat = data.site_xmat[model.site("imu").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        obs = np.hstack([
            linvel, gyro, gravity, joint_angles, joint_velocities,
            self._last_action, self._command,
        ]).astype(np.float32)
        result = self._policy.run(["continuous_actions"], {"obs": obs.reshape(1, -1)})[0][0]
        self._last_action = result.copy()
        data.ctrl[:] = result * self._action_scale + self._default_angles


class _G1Policy:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, policy_path: str, ctrl_dt: float = 0.02) -> None:
        import onnxruntime as ort
        self._policy = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
        self._default_angles = np.array(model.keyframe("home").qpos[7:])
        self._last_action = np.zeros_like(self._default_angles, dtype=np.float32)
        self._action_scale = 0.5
        self._command = np.zeros(3, dtype=np.float32)
        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.5
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt
        self._drift_compensation = np.array([-0.18, 0.0, -0.09], dtype=np.float32)

    def set_command(self, fwd: float, left: float, turn: float) -> None:
        self._command[:] = [fwd, left, turn]

    def step(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        linvel = data.sensor("local_linvel_pelvis").data
        gyro = data.sensor("gyro_pelvis").data
        imu_xmat = data.site_xmat[model.site("imu_in_pelvis").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:] - self._default_angles
        joint_velocities = data.qvel[6:]
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        cmd = self._command.copy()
        cmd[0] = cmd[0] * 2 + self._drift_compensation[0]
        cmd[1] = cmd[1] * 2 + self._drift_compensation[1]
        cmd[2] += self._drift_compensation[2]
        obs = np.hstack([
            linvel, gyro, gravity, cmd, joint_angles, joint_velocities,
            self._last_action, phase,
        ]).astype(np.float32)
        result = self._policy.run(["continuous_actions"], {"obs": obs.reshape(1, -1)})[0][0]
        self._last_action = result.copy()
        data.ctrl[:] = result * self._action_scale + self._default_angles
        self._phase = np.fmod(self._phase + self._phase_dt + np.pi, 2 * np.pi) - np.pi


class _DroneController:
    """Simple PID position controller for simulated quadrotor."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._target = np.array([0.0, 0.0, 1.5])
        self._target_yaw = 0.0
        self._command = np.zeros(3, dtype=np.float32)
        self._hover_thrust = 9.81 * self._total_mass(model, data)

    @staticmethod
    def _total_mass(model: mujoco.MjModel, data: mujoco.MjData) -> float:
        return sum(model.body_mass)

    def set_command(self, fwd: float, left: float, turn: float) -> None:
        self._command[:] = [fwd, left, turn]
        self._target[0] += fwd * 0.05
        self._target[1] += left * 0.05
        self._target_yaw += turn * 0.02

    def set_waypoint(self, x: float, y: float, z: float) -> None:
        self._target = np.array([x, y, max(z, 0.3)])

    def set_altitude(self, alt: float) -> None:
        self._target[2] = max(alt, 0.3)

    def step(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        pos = data.qpos[0:3]
        vel = data.qvel[0:3]
        err = self._target - pos
        kp, kd = 8.0, 4.0
        force = kp * err - kd * vel
        force[2] += self._hover_thrust
        n_act = model.nu
        if n_act >= 4:
            base = force[2] / 4.0
            data.ctrl[0] = base + force[0] * 0.5
            data.ctrl[1] = base - force[0] * 0.5
            data.ctrl[2] = base + force[1] * 0.5
            data.ctrl[3] = base - force[1] * 0.5
        elif n_act >= 3:
            data.ctrl[0] = force[0]
            data.ctrl[1] = force[1]
            data.ctrl[2] = force[2]


class SimulatorRunner:

    def __init__(self) -> None:
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._policy: _Go1Policy | _G1Policy | None = None
        self._lock = RLock()
        self._should_stop = Event()
        self._should_reset = Event()
        self._thread: Thread | None = None
        self._robot_id: str = ""
        self._fps: float = 0.0
        self._frame_count: int = 0
        self._state: str = "idle"
        self._sim_time: float = 0.0

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def list_robots(self) -> list[dict]:
        robots = []
        for rid, info in ROBOT_CATALOG.items():
            xml_path = ROBOTS_DIR / info["xml"]
            has_policy = info.get("policy") and (POLICIES_DIR / info["policy"]).exists()
            robots.append({
                "id": rid,
                "name": info["name"],
                "type": info["type"],
                "available": xml_path.exists(),
                "has_policy": has_policy,
            })
        return robots

    def start(self, robot_id: str = "unitree_go1", width: int = 960, height: int = 540) -> dict[str, Any]:
        if self.is_running:
            return {"ok": True, "already_running": True}

        if robot_id not in ROBOT_CATALOG:
            return {"ok": False, "error": f"Unknown robot: {robot_id}"}

        try:
            model, data, info = _load_model(robot_id)
            model.vis.global_.offwidth = max(width, model.vis.global_.offwidth)
            model.vis.global_.offheight = max(height, model.vis.global_.offheight)
            self._model = model
            self._data = data
        except Exception as e:
            return {"ok": False, "error": str(e)}

        policy_file = info.get("policy")
        controller_type = info.get("controller")
        self._policy = None
        self._drone_ctrl: _DroneController | None = None
        if controller_type == "drone":
            self._drone_ctrl = _DroneController(self._model, self._data)
            self._n_substeps = max(1, round(0.02 / float(self._model.opt.timestep)))
        elif policy_file and controller_type:
            policy_path = POLICIES_DIR / policy_file
            if policy_path.exists():
                try:
                    sim_dt = float(self._model.opt.timestep)
                    ctrl_dt = 0.02
                    n_substeps = max(1, round(ctrl_dt / sim_dt))
                    if controller_type == "go1":
                        self._policy = _Go1Policy(self._model, self._data, str(policy_path))
                    elif controller_type == "g1":
                        self._policy = _G1Policy(self._model, self._data, str(policy_path), ctrl_dt=ctrl_dt)
                    self._n_substeps = n_substeps
                except Exception:
                    self._policy = None

        if not self._policy and not self._drone_ctrl:
            self._model.opt.gravity[:] = 0

        self._robot_id = robot_id
        self._frame_count = 0
        self._fps = 0.0
        self._sim_time = 0.0
        self._should_stop.clear()
        self._state = "running"

        self._thread = Thread(
            target=self._sim_loop, args=(width, height),
            daemon=True, name="SimulatorRunner",
        )
        self._thread.start()
        return {"ok": True, "robot": robot_id, "resolution": f"{width}x{height}", "has_policy": self._policy is not None}

    def stop(self) -> dict[str, Any]:
        self._should_stop.set()
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        self._state = "idle"
        return {"ok": True}

    def reset(self) -> dict[str, Any]:
        if not self.is_running:
            return {"ok": False, "error": "Simulator not running"}
        self._should_reset.set()
        return {"ok": True}

    def set_cmd_vel(self, forward: float = 0, left: float = 0, turn: float = 0) -> None:
        if self._drone_ctrl:
            self._drone_ctrl.set_command(forward, left, turn)
        elif self._policy:
            self._policy.set_command(forward, left, turn)

    def get_state(self) -> dict[str, Any]:
        pos = [0.0, 0.0, 0.0]
        quat = [1.0, 0.0, 0.0, 0.0]
        if self._data is not None:
            pos = self._data.qpos[0:3].tolist()
            if len(self._data.qpos) >= 7:
                quat = self._data.qpos[3:7].tolist()
        info = ROBOT_CATALOG.get(self._robot_id, {})
        return {
            "running": self.is_running,
            "state": self._state,
            "robot": self._robot_id,
            "robot_type": info.get("type", ""),
            "fps": round(self._fps, 1),
            "frame_count": self._frame_count,
            "sim_time": round(self._sim_time, 2),
            "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "orientation": {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]},
            "has_policy": self._policy is not None,
            "has_drone_ctrl": self._drone_ctrl is not None,
        }

    def get_telemetry(self) -> dict[str, Any]:
        if self._data is None:
            return {}
        pos = self._data.qpos[0:3].tolist()
        quat = self._data.qpos[3:7].tolist() if len(self._data.qpos) >= 7 else [1, 0, 0, 0]
        linvel = self._data.qvel[0:3].tolist() if len(self._data.qvel) >= 3 else [0, 0, 0]
        angvel = self._data.qvel[3:6].tolist() if len(self._data.qvel) >= 6 else [0, 0, 0]
        w, x, y, z = quat
        roll = float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = float(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
        yaw = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        battery = max(0.0, 100.0 - self._sim_time / 6.0)
        joints = self._data.qpos[7:].tolist() if len(self._data.qpos) > 7 else []
        joint_vel = self._data.qvel[6:].tolist() if len(self._data.qvel) > 6 else []
        return {
            "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "orientation": {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]},
            "euler": {"roll": roll, "pitch": pitch, "yaw": yaw},
            "linear_velocity": {"x": linvel[0], "y": linvel[1], "z": linvel[2]},
            "angular_velocity": {"x": angvel[0], "y": angvel[1], "z": angvel[2]},
            "battery": round(battery, 1),
            "altitude": pos[2],
            "joint_positions": joints,
            "joint_velocities": joint_vel,
            "sim_time": self._sim_time,
        }

    def set_waypoint(self, x: float, y: float, z: float) -> dict[str, Any]:
        if self._drone_ctrl:
            self._drone_ctrl.set_waypoint(x, y, z)
            return {"ok": True}
        return {"ok": False, "error": "Not a drone"}

    def set_altitude(self, alt: float) -> dict[str, Any]:
        if self._drone_ctrl:
            self._drone_ctrl.set_altitude(alt)
            return {"ok": True}
        return {"ok": False, "error": "Not a drone"}

    def _sim_loop(self, width: int, height: int) -> None:
        fps_window: list[float] = []
        target_fps = 30.0
        renderer = mujoco.Renderer(self._model, height=height, width=width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        camera.trackbodyid = 0
        camera.distance = 3.0 if not self._drone_ctrl else 6.0
        camera.elevation = -20.0 if not self._drone_ctrl else -35.0
        camera.azimuth = 180.0

        telemetry_counter = 0

        try:
            while not self._should_stop.is_set():
                t0 = time.monotonic()

                if self._should_reset.is_set():
                    self._should_reset.clear()
                    if self._model.nkey > 0:
                        mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
                    else:
                        self._data.qpos[:] = 0
                        self._data.qpos[2] = 1.5 if self._drone_ctrl else 0.3
                        self._data.qpos[3] = 1.0
                    self._data.qvel[:] = 0
                    self._data.ctrl[:] = 0
                    mujoco.mj_forward(self._model, self._data)
                    if self._policy:
                        self._policy._last_action[:] = 0
                        self._policy._command[:] = 0
                    if self._drone_ctrl:
                        self._drone_ctrl._target = np.array([0.0, 0.0, 1.5])

                with self._lock:
                    if self._drone_ctrl:
                        n = getattr(self, "_n_substeps", 4)
                        self._drone_ctrl.step(self._model, self._data)
                        for _ in range(n):
                            mujoco.mj_step(self._model, self._data)
                        self._sim_time = self._data.time
                    elif self._policy:
                        n = getattr(self, "_n_substeps", 4)
                        self._policy.step(self._model, self._data)
                        for _ in range(n):
                            mujoco.mj_step(self._model, self._data)
                        self._sim_time = self._data.time
                    else:
                        mujoco.mj_forward(self._model, self._data)
                        self._sim_time = self._sim_time + 1.0 / target_fps

                quat = self._data.qpos[3:7]
                w, x, y, z = quat
                yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                camera.lookat[:] = self._data.qpos[0:3]
                camera.azimuth = 180.0 + np.degrees(yaw)
                renderer.update_scene(self._data, camera)
                frame_rgb = renderer.render().copy()
                frame_bgr = frame_rgb[:, :, ::-1]

                annotated = self._annotate(frame_bgr)
                ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    FRAME_PATH.write_bytes(buf.tobytes())

                # Depth rendering
                try:
                    renderer.enable_depth_rendering(True)
                    renderer.update_scene(self._data, camera)
                    depth = renderer.render().copy()
                    renderer.enable_depth_rendering(False)
                    from roborun.depth import DepthProcessor
                    DepthProcessor.get().update(depth, frame_rgb)
                except Exception:
                    pass

                # Push telemetry every ~5 frames (~6Hz)
                telemetry_counter += 1
                if telemetry_counter % 5 == 0:
                    try:
                        from roborun.telemetry import TelemetryBus
                        from roborun.trajectory import TrajectoryRecorder
                        tel = self.get_telemetry()
                        bus = TelemetryBus.get()
                        bus.push("sim", "position", tel["position"])
                        bus.push("sim", "orientation", tel["euler"])
                        bus.push("sim", "velocity", tel["linear_velocity"])
                        bus.push("sim", "battery", {"percent": tel["battery"]})
                        if tel["joint_positions"]:
                            bus.push("sim", "joint_states", {
                                "positions": tel["joint_positions"],
                                "velocities": tel["joint_velocities"],
                            })
                        pos = tel["position"]
                        ori = tel["orientation"]
                        TrajectoryRecorder.get().add_pose(
                            pos["x"], pos["y"], pos["z"],
                            ori["w"], ori["x"], ori["y"], ori["z"],
                            robot_id="sim",
                        )
                    except Exception:
                        pass

                self._frame_count += 1
                elapsed = time.monotonic() - t0
                fps_window.append(elapsed)
                if len(fps_window) > 30:
                    fps_window.pop(0)
                self._fps = 1.0 / (sum(fps_window) / len(fps_window)) if fps_window else 0

                sleep_dur = max(0, (1.0 / target_fps) - elapsed)
                if sleep_dur > 0:
                    time.sleep(sleep_dur)
        except Exception:
            pass
        finally:
            renderer.close()
            self._state = "idle"

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        out = frame.copy()
        info = ROBOT_CATALOG.get(self._robot_id, {})
        name = info.get("name", self._robot_id)
        pos = self._data.qpos[0:3] if self._data is not None else [0, 0, 0]
        mode = "WALK" if self._policy else "VIEW"

        lines = [
            f"SIM | {name} | {mode}",
            f"{self._fps:.0f} fps | t={self._sim_time:.1f}s",
            f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})",
        ]

        y0 = 22
        for line in lines:
            (lw, lh), _ = cv2.getTextSize(line, _FONT, 0.5, 1)
            cv2.rectangle(out, (8, y0 - lh - 4), (14 + lw, y0 + 4), (20, 20, 20), -1)
            cv2.putText(out, line, (10, y0), _FONT, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
            y0 += lh + 10
        return out
