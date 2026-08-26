#!/usr/bin/env bash
# Provision Gazebo (gz-sim) for the live gz backend (GAZEBO_RUNNER_SPEC).
# The RoboRun gz integration is built + tested against real MuJoCo physics
# (roborun/gz_mujoco.py); this installs the actual gz binary for live worlds.
set -euo pipefail

echo "== RoboRun: Gazebo (gz-sim) setup =="

if command -v gz >/dev/null 2>&1; then
  echo "gz already installed: $(gz --version | head -1)"; exit 0
fi

case "$(uname -s)" in
  Darwin)
    echo "macOS → Homebrew (osrf/simulation tap)"
    brew tap osrf/simulation
    brew install gz-harmonic   # Harmonic LTS
    ;;
  Linux)
    echo "Linux → apt (packages.osrfoundation.org)"
    sudo apt-get update && sudo apt-get install -y curl lsb-release gnupg
    curl -s https://packages.osrfoundation.org/gazebo.gpg \
      | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
    sudo apt-get update && sudo apt-get install -y gz-harmonic
    ;;
  *) echo "Unsupported OS"; exit 1;;
esac

echo
echo "Done. Next:"
echo "  1) gz sim -v4 shapes.sdf      # smoke-test a world"
echo "  2) install ros_gz bridge so gz topics appear as ROS 2 topics"
echo "  3) roborun connect            # detects the gz world (CAPABILITY_MATRIX['gazebo'])"
echo "  4) the GzRunner code in roborun/gz.py runs unchanged against the live world"
