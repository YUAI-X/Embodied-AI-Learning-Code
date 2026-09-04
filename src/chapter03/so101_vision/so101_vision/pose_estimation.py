"""3.5 PnP、ADD/ADD-S与物体Pose到抓取Pose。"""
# 作者：宇哥的具身笔记


from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial import cKDTree

from .camera_geometry import CameraIntrinsics, project_points
from .transforms import make_transform, transform_from_rvec, transform_points


@dataclass
class PnpResult:
    """PnP结果；变换方向为Camera←Object，重投影误差单位为像素。"""

    success: bool
    transform_camera_object: np.ndarray
    inliers: np.ndarray
    reprojection_rmse: float


def solve_pose_pnp(
    model_points: np.ndarray,
    image_points: np.ndarray,
    intrinsics: CameraIntrinsics,
    distortion: np.ndarray | None = None,
    reprojection_threshold: float = 3.0,
) -> PnpResult:
    """用RANSAC+PnP从2D-3D对应估计T_camera_object。

    model_points单位为米，image_points为(u,v)像素；RMSE只统计RANSAC内点。
    """
    distortion = np.zeros(5) if distortion is None else np.asarray(distortion)
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.asarray(model_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
        intrinsics.matrix,
        distortion,
        reprojectionError=reprojection_threshold,
        iterationsCount=200,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return PnpResult(False, np.eye(4), np.empty(0, dtype=int), float("inf"))
    transform = transform_from_rvec(rvec, tvec)
    projected = project_points(transform_points(transform, model_points), intrinsics)
    inlier_indices = inliers.reshape(-1)
    # RANSAC已经判定为外点的错误对应不应再混入重投影质量指标。
    residual = projected[inlier_indices] - np.asarray(image_points)[inlier_indices]
    rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return PnpResult(True, transform, inlier_indices, rmse)


def add_error(model_points: np.ndarray, estimate: np.ndarray, truth: np.ndarray) -> float:
    """计算非对称物体同名模型点的平均距离ADD，单位继承模型点。"""
    estimated = transform_points(estimate, model_points)
    expected = transform_points(truth, model_points)
    return float(np.linalg.norm(estimated - expected, axis=1).mean())


def adds_error(model_points: np.ndarray, estimate: np.ndarray, truth: np.ndarray) -> float:
    """对称物体用最近邻匹配，而不是强制同名模型点对应。"""
    estimated = transform_points(estimate, model_points)
    expected = transform_points(truth, model_points)
    distances, _ = cKDTree(expected).query(estimated)
    return float(distances.mean())


def cube_model(size: float = 0.06) -> np.ndarray:
    """返回以原点为中心、边长为size米的八个立方体角点。"""
    half = size / 2.0
    return np.array(
        [[x, y, z] for x in (-half, half) for y in (-half, half) for z in (-half, half)],
        dtype=float,
    )


def make_demo_pose() -> np.ndarray:
    """构造位于相机前方的可重复物体位姿真值。"""
    rvec = np.radians([12.0, -18.0, 24.0])
    rotation, _ = cv2.Rodrigues(rvec)
    return make_transform(rotation, [0.04, -0.03, 0.52])
