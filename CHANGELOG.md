# Changelog

## v0.9.2 — 2026-06-09

### Fixed
- Removed 80+ lines of dead CSS from old chat panel (`.chat-msgs`, `.msg-user`, `.msg-agent`, `.text-block`, `.thinking-block`, `.tool-call-card`, `.tool-result-card`, and related selectors)
- Fixed undefined CSS variables `--fg` → `--ink`, `--r1` → `--r` in telemetry and drone control styles
- Fixed old "RoboRun" branding in app.js header comment

## v0.7.0 — 2026-06-08

### Added
- **Unified ROS MCP** — 23 tools with DDS direct + rosbridge fallback
- Dual transport: CycloneDDS for zero-config discovery/pub, rosbridge for services/actions/params
- `scan_robots` — auto-discover any ROS robot on the network, no config
- `connect_to_robot` — dynamic rosbridge connection from the LLM
- `get_robot_info` — auto-classify robot type from topics (quadruped, drone, humanoid, arm)
- Full topic tools: `list_topics`, `get_topic_type`, `get_message_details`, `subscribe_once`, `subscribe_for_duration`, `publish`
- Service tools: `get_services`, `get_service_type`, `call_service`
- Action tools: `get_actions`, `send_action_goal`
- Parameter tools: `get_parameters`, `get_parameter`, `set_parameter`
- Node introspection: `get_nodes`
- Movement: `move` (with timed stop), `estop`, `navigate` (Nav2 goal)
- `camera_snapshot` — tries local webcam, falls back to robot camera
- `telemetry_stream` — ros_tap integration for data capture
- Main `/mcp` endpoint now includes all ROS tools (38 total)
- Dedicated `/mcp/ros` endpoint for ROS-only tools (23 total)
- `ros` optional dependency group (`pip install ros-agent[ros]`)

## v0.6.0 — 2026-06-08

### Added
- 3D scene builder with visual odometry and accumulated point cloud
- ROS telemetry module for bridging robot telemetry data
- Scene builder backend: depth-based point cloud, ORB feature matching, camera pose estimation
- Stats overlay for 3D scene (point count, keyframe count)
- Cache-busting headers on static file serving

### Fixed
- 3D scene panel rendering — Three.js canvas now initializes reliably
- Null reference crash in command center socket handler when map elements are absent
- Scene refresh polling now starts immediately on BUILD, independent of Three.js init

### Changed
- Bumped version to 0.6.0

## v0.5.0

- 3D spatial perception, drone support, telemetry dashboard

## v0.4.0

- MCP server, native ROS 2 transport, fast SDK agent, ZK prover

## v0.3.0

- Simulator, spatial memory, walking robots
