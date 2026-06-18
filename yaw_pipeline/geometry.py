"""
Geometry helpers for yaw estimation (numpy only).

Yaw convention used everywhere in this pipeline
------------------------------------------------
We define the inter-frame "yaw" as the rotation about the camera's own vertical
axis (the optical-frame y / "down" axis: x=right, y=down, z=forward). The
relative rotation between frame A and frame B, expressed in A's camera frame, is

    R_rel = R_A^T @ R_B          (R_* = camera-to-world orientation)

and the yaw is the y-component of its rotation vector:

    rel_yaw = rotvec(R_rel)[1]

This definition needs no assumption about which world axis is "up", and it is
identical for ground truth, simulated gyro, ORB, and the learned model -- so the
three comparison arms are directly comparable. Accumulated yaw is the running sum
of per-pair rel_yaw (valid small-angle approximation at 30 Hz).
"""
import numpy as np

VERTICAL_AXIS = 1  # optical-frame y ("down") = camera vertical/pan axis


def quat_to_matrix(qx, qy, qz, qw):
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def matrix_to_rotvec(R):
    """Rotation matrix -> rotation vector (axis * angle), radians."""
    cos = (np.trace(R) - 1.0) / 2.0
    cos = np.clip(cos, -1.0, 1.0)
    angle = np.arccos(cos)
    if angle < 1e-8:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # near 180 deg: use the symmetric part
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0, 1))
        # fix signs from off-diagonal
        if R[2, 1] - R[1, 2] < 0: axis[0] = -axis[0]
        if R[0, 2] - R[2, 0] < 0: axis[1] = -axis[1]
        if R[1, 0] - R[0, 1] < 0: axis[2] = -axis[2]
        return angle * axis / (np.linalg.norm(axis) + 1e-12)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return angle * ax / (2.0 * np.sin(angle))


def relative_yaw(R_a, R_b):
    """Inter-frame yaw (rad) about the camera vertical axis: rotvec(R_a^T R_b)[y]."""
    R_rel = R_a.T @ R_b
    return float(matrix_to_rotvec(R_rel)[VERTICAL_AXIS])


def wrap_angle(a):
    """Wrap radians to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def angle_to_sincos(a):
    return np.array([np.sin(a), np.cos(a)], dtype=np.float32)


def sincos_to_angle(s, c):
    return float(np.arctan2(s, c))
