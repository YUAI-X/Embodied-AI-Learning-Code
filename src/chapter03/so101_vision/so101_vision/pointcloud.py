"""3.3 轻量点云处理：体素、离群点、RANSAC、聚类、PCA和ICP。"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from .transforms import make_transform, transform_points


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """每个体素保留质心，而不是随机删点。"""
    points = np.asarray(points, dtype=float)
    if voxel_size <= 0.0:
        raise ValueError("voxel_size必须大于0")
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((inverse.max() + 1, 3), dtype=float)
    counts = np.bincount(inverse)
    np.add.at(sums, inverse, points)
    return sums / counts[:, None]


def statistical_outlier_filter(
    points: np.ndarray, neighbors: int = 16, std_ratio: float = 2.0
) -> tuple[np.ndarray, np.ndarray]:
    """根据每个点到近邻的平均距离删除孤立噪声。"""
    points = np.asarray(points, dtype=float)
    if len(points) <= neighbors:
        return points.copy(), np.ones(len(points), dtype=bool)
    distances, _ = cKDTree(points).query(points, k=neighbors + 1)
    mean_distance = distances[:, 1:].mean(axis=1)
    threshold = mean_distance.mean() + std_ratio * mean_distance.std()
    keep = mean_distance <= threshold
    return points[keep], keep


def ransac_plane(
    points: np.ndarray,
    distance_threshold: float = 0.006,
    iterations: int = 300,
    seed: int = 7,
) -> tuple[np.ndarray, np.ndarray]:
    """返回归一化平面[a,b,c,d]及内点掩码。"""
    points = np.asarray(points, dtype=float)
    if len(points) < 3:
        raise ValueError("拟合平面至少需要3个点")
    rng = np.random.default_rng(seed)
    best_model = None
    best_inliers = np.zeros(len(points), dtype=bool)
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -normal @ sample[0]
        inliers = np.abs(points @ normal + offset) <= distance_threshold
        if inliers.sum() > best_inliers.sum():
            best_model = np.r_[normal, offset]
            best_inliers = inliers
    if best_model is None:
        raise RuntimeError("RANSAC未找到有效平面")
    if best_model[2] < 0.0:
        best_model = -best_model
    return best_model, best_inliers


def euclidean_clusters(
    points: np.ndarray,
    distance: float = 0.025,
    min_points: int = 20,
    max_points: int | None = None,
) -> list[np.ndarray]:
    """使用半径邻接的欧式聚类，返回每个聚类的点索引。"""
    points = np.asarray(points, dtype=float)
    tree = cKDTree(points)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []
    for start in range(len(points)):
        if visited[start]:
            continue
        queue = [start]
        visited[start] = True
        indices = []
        while queue:
            current = queue.pop()
            indices.append(current)
            for neighbor in tree.query_ball_point(points[current], distance):
                if not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append(neighbor)
        if len(indices) >= min_points and (
            max_points is None or len(indices) <= max_points
        ):
            clusters.append(np.asarray(indices, dtype=int))
    return sorted(clusters, key=len, reverse=True)


@dataclass
class OrientedBoundingBox:
    """PCA有向包围盒；center/extent单位为米，rotation的列为主轴。"""

    center: np.ndarray
    rotation: np.ndarray
    extent: np.ndarray


def pca_bounding_box(points: np.ndarray) -> OrientedBoundingBox:
    """利用PCA主轴构造OBB；对称物体的轴方向可能跳变。"""
    points = np.asarray(points, dtype=float)
    mean = points.mean(axis=0)
    covariance = np.cov(points - mean, rowvar=False)
    values, vectors = np.linalg.eigh(covariance)
    rotation = vectors[:, np.argsort(values)[::-1]]
    if np.linalg.det(rotation) < 0.0:
        rotation[:, -1] *= -1.0
    local = (points - mean) @ rotation
    minimum, maximum = local.min(axis=0), local.max(axis=0)
    center = mean + rotation @ ((minimum + maximum) / 2.0)
    return OrientedBoundingBox(center, rotation, maximum - minimum)


@dataclass
class IcpResult:
    """点到点ICP结果；transform将Source变换到Target，rmse单位为米。"""

    transform: np.ndarray
    rmse: float
    iterations: int
    converged: bool


def _best_fit_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """用SVD求已配对点集之间的最小二乘刚体变换。"""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return make_transform(rotation, translation)


def icp_point_to_point(
    source: np.ndarray,
    target: np.ndarray,
    initial: np.ndarray | None = None,
    max_iterations: int = 40,
    tolerance: float = 1e-7,
    max_correspondence: float = 0.08,
) -> IcpResult:
    """点到点ICP；它只做局部精修，因此需要合理初值。"""
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    transform = np.eye(4) if initial is None else np.asarray(initial, dtype=float).copy()
    tree = cKDTree(target)
    previous_error = np.inf
    for iteration in range(1, max_iterations + 1):
        transformed = transform_points(transform, source)
        distances, indices = tree.query(transformed)
        valid = distances < max_correspondence
        if valid.sum() < 3:
            return IcpResult(transform, float("inf"), iteration, False)
        increment = _best_fit_transform(transformed[valid], target[indices[valid]])
        transform = increment @ transform
        error = float(np.sqrt(np.mean(distances[valid] ** 2)))
        if abs(previous_error - error) < tolerance:
            return IcpResult(transform, error, iteration, True)
        previous_error = error
    return IcpResult(transform, previous_error, max_iterations, False)
