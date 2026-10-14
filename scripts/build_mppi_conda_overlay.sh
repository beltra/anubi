#!/usr/bin/env bash
# Build nav2_mppi_controller from source into an overlay for RoboStack (conda) Humble envs.
#
# The RoboStack ros-humble-nav2-mppi-controller 1.1.17 binary (build _7) computes wrong
# controls: with a straight path ahead and every critic but PathFollowCritic disabled it
# commands steady reverse (~-0.25 m/s). The same version built from source with this
# config drives forward. The env ships xtensor 0.23.9 with xsimd 14, which cannot compile
# the upstream SIMD path, so the overlay is built with xsimd disabled.
# Docker/apt Humble is not affected; do not use this there.
#
# The overlay is sourced on every `micromamba activate` of the env through a conda
# activate.d hook, so it shadows the broken plugin for every wrapper in this workspace.
set -euo pipefail

NAV2_TAG="1.1.17"
OVERLAY_DIR="${HOME}/.local/share/anubi/mppi_overlay_ws"
INSTALL_HOOK="true"

usage() {
  cat <<'USAGE'
Usage:
  build_mppi_conda_overlay.sh [options]

Run inside the activated RoboStack env (CONDA_PREFIX must be set).

Options:
  --overlay-dir <path>  Overlay workspace (default: ~/.local/share/anubi/mppi_overlay_ws)
  --no-hook             Do not install the conda activate.d hook; source
                        <overlay-dir>/install/setup.bash yourself before the workspace
  -h, --help            Show this help
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --overlay-dir) OVERLAY_DIR="$2"; shift 2 ;;
    --no-hook) INSTALL_HOOK="false"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "${CONDA_PREFIX:-}" ]]; then
  echo "CONDA_PREFIX is not set: activate the RoboStack env first." >&2
  exit 1
fi

SRC_DIR="${OVERLAY_DIR}/src/navigation2"
mkdir -p "${OVERLAY_DIR}/src"
if [[ ! -d "${SRC_DIR}/nav2_mppi_controller" ]]; then
  git clone -q --depth 1 --branch "${NAV2_TAG}" --filter=blob:none --sparse \
    https://github.com/ros-navigation/navigation2.git "${SRC_DIR}"
  git -C "${SRC_DIR}" sparse-checkout set nav2_mppi_controller
fi

CMAKE_FILE="${SRC_DIR}/nav2_mppi_controller/CMakeLists.txt"
sed -i \
  -e 's/^add_definitions(-DXTENSOR_ENABLE_XSIMD)/# &/' \
  -e 's/^add_definitions(-DXTENSOR_USE_XSIMD)/# &/' \
  -e 's/^set(XTENSOR_USE_XSIMD 1)/set(XTENSOR_USE_XSIMD 0)/' \
  -e 's/^find_package(xsimd REQUIRED)/# &/' \
  -e 's/ xtensor::use_xsimd//' \
  "${CMAKE_FILE}"

set +u
# shellcheck disable=SC1091
source "${CONDA_PREFIX}/setup.bash" 2>/dev/null || true
set -u
(
  cd "${OVERLAY_DIR}"
  CXXFLAGS="-include cstdint" colcon build --packages-select nav2_mppi_controller \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
)

if [[ "${INSTALL_HOOK}" == "true" ]]; then
  HOOK_DIR="${CONDA_PREFIX}/etc/conda/activate.d"
  mkdir -p "${HOOK_DIR}"
  cat > "${HOOK_DIR}/zz_anubi_mppi_overlay.sh" <<HOOK
# Installed by anubi scripts/build_mppi_conda_overlay.sh: source-built MPPI overlay.
if [ -f "${OVERLAY_DIR}/install/setup.bash" ]; then
  _anubi_nounset=\$(set -o | awk '/^nounset/ {print \$2}')
  set +u
  . "${OVERLAY_DIR}/install/setup.bash"
  [ "\${_anubi_nounset}" = "on" ] && set -u
  unset _anubi_nounset
fi
HOOK
  echo "Installed activate hook: ${HOOK_DIR}/zz_anubi_mppi_overlay.sh (re-activate the env)."
else
  echo "Source ${OVERLAY_DIR}/install/setup.bash before the workspace install/setup.bash."
fi
