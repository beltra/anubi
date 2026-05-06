#!/usr/bin/env python3
"""Legacy placeholder for the LiDAR processor.

The runtime implementation is the C++ executable built from
src/autonomy/robot_nav/src/distortion_corrector.cpp. This file is intentionally
not installed by CMake so the package has one active processing path.
"""


def main() -> None:
    raise SystemExit(
        "distortion_corrector.py is not a runtime node; use the C++ "
        "robot_nav/distortion_corrector executable."
    )


if __name__ == "__main__":
    main()
