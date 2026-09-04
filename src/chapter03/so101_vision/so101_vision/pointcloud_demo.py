"""3.3 Demo：桌面点云预处理、平面分割、聚类和OBB。"""
# 作者：宇哥的具身笔记


import argparse

import numpy as np

from .pointcloud import (
    euclidean_clusters,
    pca_bounding_box,
    ransac_plane,
    statistical_outlier_filter,
    voxel_downsample,
)
from .synthetic_data import synthetic_tabletop_cloud


def main():
    """执行桌面点云下采样、去噪、平面分割、聚类与OBB估计。"""
    parser = argparse.ArgumentParser(description="无硬件桌面点云处理")
    parser.add_argument("--voxel", type=float, default=0.008)
    parser.add_argument("--plane-threshold", type=float, default=0.006)
    parser.add_argument("--cluster-distance", type=float, default=0.025)
    args = parser.parse_args()

    raw, _ = synthetic_tabletop_cloud()
    downsampled = voxel_downsample(raw, args.voxel)
    filtered, _ = statistical_outlier_filter(downsampled, 16, 2.0)
    plane, table_mask = ransac_plane(filtered, args.plane_threshold)
    objects = filtered[~table_mask & (filtered[:, 2] > 0.008)]
    clusters = euclidean_clusters(objects, args.cluster_distance, min_points=12)

    print(f"原始点数：{len(raw)}")
    print(f"体素下采样：{len(downsampled)}")
    print(f"离群点过滤后：{len(filtered)}")
    print("桌面平面[a,b,c,d]：", np.round(plane, 5))
    print(f"物体聚类数：{len(clusters)}")
    for index, indices in enumerate(clusters, 1):
        box = pca_bounding_box(objects[indices])
        print(
            f"  物体{index}: 点数={len(indices)}, 中心={np.round(box.center, 3)}, "
            f"尺寸={np.round(np.sort(box.extent), 3)}"
        )
    print("提示：改变体素、平面阈值和聚类距离，观察欠分割与过分割。")


if __name__ == "__main__":
    main()
