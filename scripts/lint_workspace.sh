#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${workspace_root}"

echo "[lint] Python compile check"
python3 -m compileall -q \
  scripts \
  src/autonomy \
  src/core_interfaces \
  src/platform \
  src/simulation

if command -v flake8 >/dev/null 2>&1; then
  echo "[lint] flake8 first-party Python"
  flake8 \
    --max-line-length=100 \
    --ignore=E203,W503 \
    --select=E9,F63,F7,F82 \
    --exclude='build,install,log,__pycache__,src/vendors' \
    scripts \
    src/autonomy \
    src/simulation
else
  echo "[lint] flake8 not installed; skipping"
fi

if command -v cppcheck >/dev/null 2>&1; then
  echo "[lint] cppcheck first-party C++"
  cppcheck \
    --enable=warning,portability \
    --error-exitcode=1 \
    --suppress=missingIncludeSystem \
    --inline-suppr \
    src/platform/go2_bringup/src \
    src/platform/go2_bringup/include
else
  echo "[lint] cppcheck not installed; skipping"
fi

echo "[lint] OK"
