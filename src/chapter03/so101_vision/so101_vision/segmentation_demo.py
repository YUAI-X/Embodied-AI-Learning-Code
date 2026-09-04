"""3.4 Demo：文本目标→Mask→目标三维点云。"""
# 作者：宇哥的具身笔记


import argparse

import cv2
import numpy as np

from .segmentation import mask_bounding_box, masked_target_cloud, segment_color_prompt
from .synthetic_data import synthetic_rgbd


def main():
    """演示文本颜色目标到Mask、检测框和目标点云的完整数据流。"""
    parser = argparse.ArgumentParser(description="轻量Grounded-SAM流程替身")
    parser.add_argument("--prompt", default="red cup")
    parser.add_argument("--save", default="")
    args = parser.parse_args()
    color, depth, intrinsics, _ = synthetic_rgbd()
    mask = segment_color_prompt(color, args.prompt)
    box = mask_bounding_box(mask)
    points, _, eroded = masked_target_cloud(color, depth, mask, intrinsics)
    median = np.median(points, axis=0)
    minimum, maximum = points.min(axis=0), points.max(axis=0)
    print("Prompt：", args.prompt)
    print("检测框[x1,y1,x2,y2]：", box)
    print("Mask像素数/腐蚀后：", np.count_nonzero(mask), np.count_nonzero(eroded))
    print("目标有效三维点数：", len(points))
    print("目标三维中位点 Camera Frame(m)：", np.round(median, 4))
    print("目标点云AABB尺寸(m)：", np.round(maximum - minimum, 4))
    if args.save:
        visualization = color.copy()
        visualization[eroded == 0] //= 3
        cv2.rectangle(visualization, box[:2], box[2:], (0, 255, 255), 2)
        cv2.imwrite(args.save, visualization)
        print("可视化已保存：", args.save)


if __name__ == "__main__":
    main()
