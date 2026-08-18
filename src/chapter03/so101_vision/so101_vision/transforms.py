"""3.2/3.5 齐次变换、坐标链和位姿误差。"""

import cv2
import numpy as np


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """用3×3旋转和三维平移构造T_parent_child。"""
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def transform_from_rvec(rvec: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """把OpenCV旋转向量和平移向量组合成4×4齐次变换。"""
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float).reshape(3, 1))
    return make_transform(rotation, translation)


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """求T_A_B的逆T_B_A，平移不能只简单取负。"""
    transform = np.asarray(transform, dtype=float)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    result = np.eye(4)
    result[:3, :3] = rotation.T
    result[:3, 3] = -rotation.T @ translation
    return result


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """对N×3点批量应用齐次变换，返回同形状笛卡尔坐标。"""
    points = np.asarray(points, dtype=float)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    """计算两个齐次位姿旋转部分之间的最小夹角，单位为度。"""
    relative = first[:3, :3] @ second[:3, :3].T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))
