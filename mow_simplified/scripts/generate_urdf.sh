#!/usr/bin/env bash
# Regenera urdf/mow_simplified.urdf desde urdf/mow_simplified.urdf.xacro (hace falta `xacro`: ROS 2 o `pip install xacro`).
# Correr cada vez que cambie ros2_ws/src/lawnbot_nav/config/robot_geometry.yaml (corner_sonar_angle_deg).
set -euo pipefail
cd "$(dirname "$0")/.."
GEOMETRY="$(cd .. && pwd)/ros2_ws/src/lawnbot_nav/config/robot_geometry.yaml"
xacro urdf/mow_simplified.urdf.xacro geometry_file:="$GEOMETRY" -o urdf/mow_simplified.urdf
echo "urdf/mow_simplified.urdf regenerado"
