# Changelog

## v0.7.0 — 2026-06-08

### Added
- **Direct ROS MCP server** at `/mcp/ros` — direct DDS connectivity, no rosbridge or ROS install needed
- Robot auto-discovery via ros_tap CycloneDDS (scan network, list all topics/nodes)
- Direct `/cmd_vel` publishing over DDS for move/estop commands
- 10 MCP tools: scan_robots, list_topics, read_topic, publish_topic, move, estop, camera_snapshot, battery_status, navigate, telemetry_stream
- `ros_tap` integration for zero-config telemetry streaming
- `ros` optional dependency group (`pip install roborun[ros]`)

### Changed
- Updated project description
- MCP server version bumped to 0.7.0

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
