"""
pointcloud_utils.py
Shared utilities for LiDAR pointcloud processing nodes (distortion_corrector,
cloud_self_filter).

All functions are pure numpy; no ROS imports.
"""

import math

import numpy as np


def stamp_to_sec(stamp) -> float:
    """Convert a ROS header stamp to seconds (float)."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


# Rotation helpers

def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return the 3x3 rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr = math.cos(roll);  sr = math.sin(roll)
    cp = math.cos(pitch); sp = math.sin(pitch)
    cy = math.cos(yaw);   sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
            [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
            [-sp,      cp * sr,                 cp * cr               ],
        ],
        dtype=np.float64,
    )


def roll_pitch_from_imu(quat) -> tuple[float, float]:
    """Extract roll and pitch (radians) from a ROS Imu orientation quaternion.

    Yaw is discarded - only the gravity-correcting components are returned.
    quat must expose .w, .x, .y, .z attributes (sensor_msgs/Imu.orientation).
    """
    w, x, y, z = quat.w, quat.x, quat.y, quat.z
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    return roll, pitch


# PointCloud2 parsing

def build_pc2_dtype(msg) -> np.dtype:
    """Build a numpy structured dtype from a sensor_msgs/PointCloud2 message.

    Accounts for padding between fields and at the end of each point so that
    the dtype itemsize equals msg.point_step.  Without this, np.frombuffer
    misaligns every point after the first.
    """
    _ROS_TO_NP = {
        1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
        5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
    }
    fields: dict = {}
    for f in msg.fields:
        np_type = _ROS_TO_NP.get(f.datatype, np.uint8)
        if f.count == 1:
            fields[f.name] = (np_type, f.offset)
        else:
            fields[f.name] = (np_type, (f.count,), f.offset)
    return np.dtype(
        {
            "names":    list(fields.keys()),
            "formats":  [v[0] if len(v) == 2 else (v[0], v[1]) for v in fields.values()],
            "offsets":  [v[-1] for v in fields.values()],
            "itemsize": msg.point_step,
        }
    )


# Gravity alignment

def gravity_align_cloud(
    cx: np.ndarray,
    cy: np.ndarray,
    cz: np.ndarray,
    finite_mask: np.ndarray,
    R_body: np.ndarray,
    R_mount_inv: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate a pointcloud from lidar_link to the gravity-levelled frame.

    Applies: P_stab = R_body @ R_mount_inv @ P_lidar_link

    R_mount_inv  - inverse of the static base_link -> lidar_link rotation
                   (lidar_link -> base_link).
    R_body       - roll+pitch body-to-world rotation from IMU (yaw=0).

    NaN points propagate through matrix multiply and are preserved by np.where,
    so the caller does not need to pre-filter.

    Returns (cx_out, cy_out, cz_out) as float32 arrays.
    """
    pts = R_mount_inv @ np.stack(
        [cx.astype(np.float64), cy.astype(np.float64), cz.astype(np.float64)],
        axis=0,
    )  # 3 x N
    pts_stab = R_body @ pts
    cx_out = np.where(finite_mask, pts_stab[0].astype(np.float32), cx)
    cy_out = np.where(finite_mask, pts_stab[1].astype(np.float32), cy)
    cz_out = np.where(finite_mask, pts_stab[2].astype(np.float32), cz)
    return cx_out, cy_out, cz_out
