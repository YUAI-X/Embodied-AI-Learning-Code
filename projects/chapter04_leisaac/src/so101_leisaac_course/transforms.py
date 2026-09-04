"""UMI实验使用的SE(3)齐次变换，不依赖ROS TF。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

import math

import numpy as np


def normalize_quaternion(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """归一化[x,y,z,w]四元数，并固定w>=0以减少符号跳变。"""
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,):
        raise ValueError("四元数必须是[x,y,z,w]四维向量")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("四元数范数不能为0")
    quaternion = quaternion / norm
    return -quaternion if quaternion[3] < 0 else quaternion


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """把[x,y,z,w]四元数转换为3×3旋转矩阵。"""
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """稳定地把3×3旋转矩阵转换为[x,y,z,w]四元数。"""
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("旋转矩阵必须是3×3")
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return normalize_quaternion(quaternion)


def pose7_to_matrix(pose_xyz_xyzw: np.ndarray) -> np.ndarray:
    """将[x,y,z,qx,qy,qz,qw]位姿转换为4×4齐次矩阵。"""
    pose = np.asarray(pose_xyz_xyzw, dtype=float)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError("位姿必须是有限的7维向量[x,y,z,qx,qy,qz,qw]")
    transform = np.eye(4)
    transform[:3, :3] = quaternion_to_matrix(pose[3:])
    transform[:3, 3] = pose[:3]
    return transform


def matrix_to_pose7(transform: np.ndarray) -> np.ndarray:
    """将4×4齐次矩阵转换回[x,y,z,qx,qy,qz,qw]。"""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("齐次变换必须是4×4")
    return np.r_[transform[:3, 3], matrix_to_quaternion(transform[:3, :3])]


def relative_transform(reference: np.ndarray, target: np.ndarray) -> np.ndarray:
    """计算T_rel=inv(T_reference)@T_target。"""
    return np.linalg.inv(np.asarray(reference, dtype=float)) @ np.asarray(target, dtype=float)


def rotation_matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    """把旋转矩阵转成轴角向量，向量长度为旋转角（弧度）。"""
    quaternion = matrix_to_quaternion(rotation)
    vector = quaternion[:3]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, quaternion[3])
    return vector / vector_norm * angle


def transform_to_action6(transform: np.ndarray) -> np.ndarray:
    """将相对SE(3)表示为[dx,dy,dz,rx,ry,rz]，旋转部分是轴角向量。"""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("齐次变换必须是4×4")
    return np.r_[transform[:3, 3], rotation_matrix_to_rotvec(transform[:3, :3])]
