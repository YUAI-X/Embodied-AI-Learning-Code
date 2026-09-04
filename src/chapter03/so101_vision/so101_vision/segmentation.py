"""3.4 轻量文本颜色检测、Mask与深度融合。"""
# 作者：宇哥的具身笔记


import cv2
import numpy as np

from .camera_geometry import CameraIntrinsics, depth_to_pointcloud


COLOR_RANGES = {
    "red": [((0, 80, 60), (10, 255, 255)), ((170, 80, 60), (180, 255, 255))],
    "blue": [((90, 60, 40), (135, 255, 255))],
    "green": [((35, 50, 40), (90, 255, 255))],
}


def color_from_prompt(prompt: str) -> str:
    """从中英文Prompt中提取轻量教学模式支持的颜色名称。"""
    normalized = prompt.lower()
    aliases = {"红": "red", "蓝": "blue", "绿": "green"}
    for chinese, english in aliases.items():
        if chinese in prompt:
            return english
    for color in COLOR_RANGES:
        if color in normalized:
            return color
    raise ValueError("轻量模式仅识别red/blue/green颜色词；真实开放词汇请使用可选模型")


def segment_color_prompt(image_bgr: np.ndarray, prompt: str) -> np.ndarray:
    """教学替身：用颜色词生成Mask，保持与真实Grounded-SAM相同输出边界。"""
    color_name = color_from_prompt(prompt)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(image_bgr.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color_name]:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def mask_bounding_box(mask: np.ndarray) -> tuple[int, int, int, int]:
    """返回非零Mask的闭区间像素框(x1,y1,x2,y2)。"""
    rows, columns = np.nonzero(mask)
    if len(rows) == 0:
        raise ValueError("Mask为空")
    return int(columns.min()), int(rows.min()), int(columns.max()), int(rows.max())


def masked_target_cloud(
    color: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    erosion_pixels: int = 2,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """轻微腐蚀Mask后生成目标点云，并用深度中位数去除背景。"""
    kernel_size = max(1, erosion_pixels * 2 + 1)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    eroded = cv2.erode(mask.astype(np.uint8), kernel)
    points, colors = depth_to_pointcloud(depth, intrinsics, color, eroded > 0)
    if len(points) == 0:
        raise ValueError("Mask内没有有效深度")
    median_z = np.median(points[:, 2])
    keep = np.abs(points[:, 2] - median_z) < 0.04
    return points[keep], colors[keep], eroded
