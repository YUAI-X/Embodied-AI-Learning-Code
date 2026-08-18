"""生成无需相机和下载数据集即可运行的RGB-D与桌面点云。"""

import cv2
import numpy as np

from .camera_geometry import CameraIntrinsics


def default_intrinsics(width: int = 320, height: int = 240) -> CameraIntrinsics:
    """返回与分辨率同步缩放的教学内参。

    320×240时fx=fy=280 px；分辨率变化时焦距像素值也必须同比变化。
    """
    return CameraIntrinsics(
        width,
        height,
        280.0 * width / 320.0,
        280.0 * height / 240.0,
        (width - 1) / 2,
        (height - 1) / 2,
    )


def synthetic_rgbd(width: int = 320, height: int = 240):
    """生成灰色桌面、红杯、蓝盒和绿色零件的对齐RGB-D。"""
    intrinsics = default_intrinsics(width, height)
    color = np.full((height, width, 3), (90, 90, 90), dtype=np.uint8)
    depth = np.full((height, width), 0.78, dtype=np.float32)
    masks = {}

    red_cup = np.zeros((height, width), dtype=np.uint8)
    cv2.ellipse(red_cup, (95, 135), (28, 45), 0, 0, 360, 255, -1)
    color[red_cup > 0] = (35, 35, 220)
    depth[red_cup > 0] = 0.46
    masks["red cup"] = red_cup

    blue_box = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(blue_box, (155, 105), (225, 180), 255, -1)
    color[blue_box > 0] = (220, 70, 30)
    depth[blue_box > 0] = 0.55
    masks["blue box"] = blue_box

    green_part = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(green_part, (260, 145), 25, 255, -1)
    color[green_part > 0] = (50, 190, 50)
    depth[green_part > 0] = 0.62
    masks["green part"] = green_part

    # 模拟无效深度和边缘飞点，供故障排查实验观察。
    depth[35:50, 40:60] = np.nan
    red_rows, red_columns = np.nonzero(red_cup)
    depth[red_rows[::19], red_columns[::19]] = 0.0
    return color, depth, intrinsics, masks


def sample_cuboid(center, size, count, rng):
    """在长方体六个表面采样点。"""
    center = np.asarray(center, dtype=float)
    size = np.asarray(size, dtype=float)
    points = rng.uniform(-0.5, 0.5, size=(count, 3)) * size
    faces = rng.integers(0, 6, size=count)
    axes = faces // 2
    signs = np.where(faces % 2 == 0, -0.5, 0.5)
    points[np.arange(count), axes] = signs * size[axes]
    return points + center


def synthetic_tabletop_cloud(seed: int = 7):
    """生成桌面、三个物体和离群噪声，并返回真实标签。"""
    rng = np.random.default_rng(seed)
    table_xy = rng.uniform([-0.35, -0.28], [0.35, 0.28], size=(4500, 2))
    table_z = rng.normal(0.0, 0.0015, size=(4500, 1))
    table = np.column_stack((table_xy, table_z))
    objects = [
        sample_cuboid([-0.16, 0.02, 0.045], [0.07, 0.07, 0.09], 900, rng),
        sample_cuboid([0.05, -0.10, 0.035], [0.10, 0.06, 0.07], 1000, rng),
        sample_cuboid([0.19, 0.11, 0.055], [0.06, 0.09, 0.11], 800, rng),
    ]
    noise = rng.uniform([-0.4, -0.35, -0.04], [0.4, 0.35, 0.18], size=(130, 3))
    points = np.vstack([table, *objects, noise])
    labels = np.concatenate(
        [
            np.zeros(len(table), dtype=int),
            np.full(len(objects[0]), 1),
            np.full(len(objects[1]), 2),
            np.full(len(objects[2]), 3),
            np.full(len(noise), -1),
        ]
    )
    return points, labels
