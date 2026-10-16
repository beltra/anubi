#!/bin/bash
# Source this (do not execute) before talking to a Unitree robot over CycloneDDS:
#   UNITREE_NET_IFACE=eth0 source scripts/unitree_setup.sh
# UNITREE_NET_IFACE must be the wired NIC on the robot LAN (192.168.123.x).
# UNITREE_ROS2_WS points at a built unitree_ros2 cyclonedds_ws.

: "${UNITREE_NET_IFACE:?set UNITREE_NET_IFACE to the NIC on the robot LAN (see ip -br addr)}"
UNITREE_ROS2_WS="${UNITREE_ROS2_WS:-$HOME/unitree_ros2/cyclonedds_ws}"

echo "Setup unitree ros2 environment (iface: ${UNITREE_NET_IFACE})"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${UNITREE_ROS2_WS}/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name=\"${UNITREE_NET_IFACE}\" priority=\"default\" multicast=\"default\" />
                        </Interfaces></General></Domain></CycloneDDS>"
