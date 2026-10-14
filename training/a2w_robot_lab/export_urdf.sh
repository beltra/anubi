#!/usr/bin/env bash
# Export the articulated A2-W URDF (legs revolute, wheels continuous) for Isaac Lab.
# Needs a sourced ROS env with a2w_description built. Mesh URIs are rewritten to relative
# paths and the meshes are copied, so the output directory is self-contained.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT_DIR="${1:-${SCRIPT_DIR}/unitree_a2w/description}"
DESC_SHARE="$(ros2 pkg prefix a2w_description)/share/a2w_description"

mkdir -p "${OUT_DIR}/urdf" "${OUT_DIR}/meshes"
xacro "${DESC_SHARE}/urdf/a2w.urdf.xacro" fixed_legs:=false \
  | sed 's#package://a2w_description/meshes/#../meshes/#g' > "${OUT_DIR}/urdf/a2w.urdf"
cp "${DESC_SHARE}"/meshes/*.STL "${OUT_DIR}/meshes/"
cp "${DESC_SHARE}/LICENSE.unitree_ros" "${OUT_DIR}/" 2>/dev/null || true
echo "Wrote ${OUT_DIR}/urdf/a2w.urdf"
