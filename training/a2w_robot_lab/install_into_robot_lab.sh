#!/usr/bin/env bash
# Link the A2-W task into a robot_lab checkout (v2.3.2) so robot_lab registers
# RobotLab-Isaac-Velocity-{Flat,Rough}-Unitree-A2W-v0. Run export_urdf.sh first.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_LAB_DIR="${1:?usage: install_into_robot_lab.sh <robot_lab checkout>}"
TASK_DIR="${ROBOT_LAB_DIR}/source/robot_lab/robot_lab/tasks/manager_based/locomotion/velocity/config/wheeled"

if [[ ! -d "${TASK_DIR}" ]]; then
  echo "Not a robot_lab checkout: ${TASK_DIR} missing" >&2
  exit 1
fi
if [[ ! -f "${SCRIPT_DIR}/unitree_a2w/description/urdf/a2w.urdf" ]]; then
  echo "Missing exported URDF: run export_urdf.sh" >&2
  exit 1
fi
ln -sfn "${SCRIPT_DIR}/unitree_a2w" "${TASK_DIR}/unitree_a2w"
echo "Linked ${TASK_DIR}/unitree_a2w -> ${SCRIPT_DIR}/unitree_a2w"
