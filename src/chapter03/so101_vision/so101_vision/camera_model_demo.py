"""3.1 Demo：像素投影、深度反投影与CameraInfo缩放。"""
# 作者：宇哥的具身笔记


import numpy as np

from .camera_geometry import backproject_pixels, project_points
from .synthetic_data import synthetic_rgbd


def main():
    """验证像素反投影与三维点重投影互为逆过程。"""
    color, depth, intrinsics, _ = synthetic_rgbd()
    # 避开合成图中故意设置为0的红杯飞点；0深度应在点云阶段被过滤。
    pixels = np.array([[96, 135], [190, 140], [260, 145]], dtype=float)
    sampled_depth = depth[pixels[:, 1].astype(int), pixels[:, 0].astype(int)]
    points = backproject_pixels(pixels, sampled_depth, intrinsics)
    roundtrip = project_points(points, intrinsics)

    print("相机内参K：\n", intrinsics.matrix)
    print("RGB尺寸：", color.shape, "Depth尺寸：", depth.shape)
    print("像素(u,v)：\n", pixels)
    print("反投影Camera Frame点(m)：\n", np.round(points, 4))
    print("重新投影像素：\n", np.round(roundtrip, 4))
    print("最大往返误差(px)：", np.abs(roundtrip - pixels).max())
    print("缩放一半后的K：\n", intrinsics.resized(160, 120).matrix)


if __name__ == "__main__":
    main()
