#!/usr/bin/env python3

from __future__ import annotations

import numpy as np


ORTHOGONALITY_TOLERANCE = 1e-3
AXIS_ALIGNMENT_WARNING_THRESHOLD = 0.95
HORIZONTAL_PROJECTION_TOLERANCE = 1e-6
WORLD_DOWN_AXIS = np.array([0.0, 0.0, -1.0], dtype=np.float64)


def normalize_vector(vector: np.ndarray, name: str) -> np.ndarray:
    norm = np.linalg.norm(vector)
    if norm == 0.0:
        raise ValueError(f"{name} must not be the zero vector.")
    return vector / norm


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to an xyzw quaternion."""
    trace = float(np.trace(rotation_matrix))

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
    else:
        diagonal = np.diag(rotation_matrix)
        max_index = int(np.argmax(diagonal))

        if max_index == 0:
            s = 2.0 * np.sqrt(
                1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qz = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif max_index == 1:
            s = 2.0 * np.sqrt(
                1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]
            )
            qw = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            qx = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(
                1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]
            )
            qw = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            qx = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            qy = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quaternion / np.linalg.norm(quaternion)


def quaternion_to_rotation_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = normalize_vector(quaternion_xyzw, "quaternion_xyzw")

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def validate_input_axes(frame_x: np.ndarray, frame_y: np.ndarray, frame_z: np.ndarray) -> None:
    if abs(float(np.dot(frame_x, frame_y))) > ORTHOGONALITY_TOLERANCE:
        raise ValueError("frame_x and frame_y must be orthogonal.")
    if abs(float(np.dot(frame_x, frame_z))) > ORTHOGONALITY_TOLERANCE:
        raise ValueError("frame_x and frame_z must be orthogonal.")
    if abs(float(np.dot(frame_y, frame_z))) > ORTHOGONALITY_TOLERANCE:
        raise ValueError("frame_y and frame_z must be orthogonal.")


def compute_gripper_quaternion(
    frame_x: np.ndarray, frame_y: np.ndarray, frame_z: np.ndarray
) -> tuple[np.ndarray, str | None]:
    validate_input_axes(frame_x, frame_y, frame_z)

    # Global top-down grasp convention:
    # - TCP z always points along -world z.
    # - TCP x follows the negated tool frame z-axis projected into the horizontal plane.
    target_z = WORLD_DOWN_AXIS
    projected_x = frame_z - np.dot(frame_z, target_z) * target_z
    projection_norm = np.linalg.norm(projected_x)
    if projection_norm < HORIZONTAL_PROJECTION_TOLERANCE:
        raise ValueError(
            "frame_z is too parallel to world z, so its horizontal projection is undefined."
        )

    target_x = -projected_x / projection_norm
    target_y = normalize_vector(np.cross(target_z, target_x), "target_y")
    alignment = float(np.dot(target_x, -frame_z))
    warning = None

    if abs(alignment) < AXIS_ALIGNMENT_WARNING_THRESHOLD:
        warning = (
            "Projected target_x differs noticeably from the expected -frame_z direction. "
            f"dot(target_x, -frame_z)={alignment:.4f}"
        )

    rotation_matrix = np.column_stack((target_x, target_y, target_z))
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)
    return quaternion, warning


def compute_gripper_quaternion_from_rotation_matrix(
    rotation_matrix: np.ndarray,
) -> tuple[np.ndarray, str | None]:
    frame_x = normalize_vector(rotation_matrix[:, 0], "frame_x")
    frame_y = normalize_vector(rotation_matrix[:, 1], "frame_y")
    frame_z = normalize_vector(rotation_matrix[:, 2], "frame_z")
    return compute_gripper_quaternion(frame_x, frame_y, frame_z)
