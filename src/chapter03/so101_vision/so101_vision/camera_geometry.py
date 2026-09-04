"""3.1 相机投影、反投影与深度图点云化。"""
# 作者：宇哥的具身笔记


from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """针孔相机内参，焦距使用像素单位。"""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def matrix(self) -> np.ndarray:
        """返回OpenCV格式的3×3内参矩阵K。"""
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=float,
        )

    def resized(self, width: int, height: int) -> "CameraIntrinsics":
        """图像缩放后同步缩放内参，避免继续使用旧K。"""
        scale_x = width / self.width
        scale_y = height / self.height
        return CameraIntrinsics(
            width,
            height,
            self.fx * scale_x,
            self.fy * scale_y,
            self.cx * scale_x,
            self.cy * scale_y,
        )


def project_points(points_camera: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    """把Camera Frame中的N×3点投影成N×2像素；Z必须为正。"""
    points = np.asarray(points_camera, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_camera必须是N×3数组")
    if np.any(points[:, 2] <= 0.0):
        raise ValueError("相机前方点的Z必须大于0")
    u = intrinsics.fx * points[:, 0] / points[:, 2] + intrinsics.cx
    v = intrinsics.fy * points[:, 1] / points[:, 2] + intrinsics.cy
    return np.column_stack((u, v))


def backproject_pixels(
    pixels: np.ndarray, depths_m: np.ndarray, intrinsics: CameraIntrinsics
) -> np.ndarray:
    """把N个像素及其米制深度反投影为Camera Frame三维点。"""
    pixels = np.asarray(pixels, dtype=float)
    depths = np.asarray(depths_m, dtype=float).reshape(-1)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) != len(depths):
        raise ValueError("pixels应为N×2，depths_m应为相同长度")
    x = (pixels[:, 0] - intrinsics.cx) * depths / intrinsics.fx
    y = (pixels[:, 1] - intrinsics.cy) * depths / intrinsics.fy
    return np.column_stack((x, y, depths))


def depth_to_pointcloud(
    depth: np.ndarray,
    intrinsics: CameraIntrinsics,
    color: np.ndarray | None = None,
    mask: np.ndarray | None = None,
    min_depth: float = 0.1,
    max_depth: float = 2.0,
) -> tuple[np.ndarray, np.ndarray | None]:
    """把32FC1米制深度图转换为点云，并过滤0、NaN和量程外深度。"""
    depth = np.asarray(depth, dtype=float)
    if depth.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("深度图尺寸必须与CameraInfo一致")
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    if mask is not None:
        valid &= np.asarray(mask, dtype=bool)
    rows, columns = np.nonzero(valid)
    pixels = np.column_stack((columns, rows))
    points = backproject_pixels(pixels, depth[rows, columns], intrinsics)
    colors = None
    if color is not None:
        color = np.asarray(color)
        if color.shape[:2] != depth.shape:
            raise ValueError("RGB与Depth必须先对齐且尺寸相同")
        colors = color[rows, columns]
    return points, colors
