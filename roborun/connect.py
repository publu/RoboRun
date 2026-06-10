"""`roborun connect <robot>` — the shortest path from a robot on your
network to a robot you can drive.

    roborun connect 192.168.1.42          # rosbridge on the robot
    roborun connect 192.168.1.42 --move   # prove it: clamped 0.5s wiggle
    roborun connect --scan                # DDS discovery (needs [ros] extra)

On success the robot is saved to ~/.roborun/robot.json and every later
plain `roborun` connects to it automatically — behaviors, deck, MCP.
On failure it prints the exact commands to run on the robot, because
"go figure out rosbridge" is the setup work this project exists to
delete.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

ROBOT_FILE = Path.home() / ".roborun" / "robot.json"

_ROSBRIDGE_HELP = """\
could not reach ws://{host}:{port} — rosbridge isn't running there.

On the robot (one time):
  ROS 2:  sudo apt install ros-$ROS_DISTRO-rosbridge-suite
          ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  ROS 1:  sudo apt install ros-$ROS_DISTRO-rosbridge-suite
          roslaunch rosbridge_server rosbridge_websocket.launch

No SSH access / no rosbridge allowed? DDS-direct needs nothing on the
robot:  pip install 'ros-agent[ros]' && roborun connect --scan"""


def saved_robot() -> dict | None:
    """The robot a previous `roborun connect` saved, or None."""
    try:
        return json.loads(ROBOT_FILE.read_text())
    except Exception:
        return None


def _save(host: str, port: int, robot_type: str, topics: int) -> None:
    ROBOT_FILE.parent.mkdir(parents=True, exist_ok=True)
    ROBOT_FILE.write_text(json.dumps(
        {"host": host, "port": port, "type": robot_type, "topics": topics,
         "connected_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())},
        indent=1))


def cli(argv: list[str] | None = None) -> int:
    import argparse
    p = argparse.ArgumentParser(
        prog="roborun connect",
        description="Connect to a robot and remember it. The goal is motion.")
    p.add_argument("host", nargs="?", help="robot IP / hostname (rosbridge)")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--move", action="store_true",
                   help="prove the connection: clamped 0.5s forward nudge + stop")
    p.add_argument("--scan", action="store_true",
                   help="DDS discovery instead of rosbridge (needs [ros] extra)")
    p.add_argument("--forget", action="store_true", help="forget the saved robot")
    args = p.parse_args(argv)

    if args.forget:
        ROBOT_FILE.unlink(missing_ok=True)
        print("forgotten — `roborun` is back to webcam + sim")
        return 0

    if args.scan:
        return _scan()
    if not args.host:
        p.error("host required (or --scan)")

    from roborun.rosbridge import RosbridgeClient
    print(f"connecting to ws://{args.host}:{args.port} …")
    try:
        client = RosbridgeClient(args.host, args.port)
        client.connect(timeout=5.0)
    except Exception:
        print(_ROSBRIDGE_HELP.format(host=args.host, port=args.port))
        return 1

    topics = client.list_topics()
    names = [t.get("topic", t.get("name", "")) for t in topics]
    from roborun.robot_types import detect_type
    rtype = detect_type(ros_topics=names).value

    _save(args.host, args.port, rtype, len(topics))
    print(f"connected · {len(topics)} topics · looks like a {rtype}")

    if args.move:
        print("wiggle test: 0.1 m/s forward for 0.5s …")
        client.move(linear_x=0.1)
        time.sleep(0.5)
        client.move(linear_x=0.0)
        print("stopped. If it moved, you're done with setup.")

    print(f"\nsaved to {ROBOT_FILE} — `roborun` now drives this robot.\n"
          f"Next: run `roborun`, then edit behaviors/follow_person.py.")
    client.disconnect()
    return 0


def _scan() -> int:
    try:
        from roborun.transport import discover
    except Exception:
        print("DDS scan needs the [ros] extra: pip install 'ros-agent[ros]'")
        return 1
    print("scanning DDS domains for robots …")
    robots = discover()
    if not robots:
        print("nothing found. Same network? Same ROS_DOMAIN_ID? "
              "rosbridge is the fallback: roborun connect <robot-ip>")
        return 1
    for r in robots:
        print(f"  {r}")
    print("connect with: roborun connect <host>")
    return 0


if __name__ == "__main__":
    raise SystemExit(cli())
