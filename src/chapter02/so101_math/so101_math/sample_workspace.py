"""随机采样关节角，绘制 SO-101 末端工作空间。"""
# 作者：宇哥的具身笔记


import argparse
import numpy as np
import matplotlib.pyplot as plt
from .kinematics import JOINT_LIMITS, forward_kinematics, numerical_position_jacobian
from .plotting import configure_chinese_font


def main():
    """在官方关节限制内采样，以米制三维点绘制SO101工作空间。"""
    selected_font = configure_chinese_font()
    parser = argparse.ArgumentParser(description="采样 SO-101 工作空间")
    parser.add_argument("--samples", type=int, default=1500)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--save", default="")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()
    rng = np.random.default_rng(args.seed)
    joints = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1], size=(args.samples, 5))
    points = np.zeros((args.samples, 3))
    condition = np.zeros(args.samples)
    for index, sample in enumerate(joints):
        points[index] = forward_kinematics(sample)[:3, 3]
        singular_values = np.linalg.svd(
            numerical_position_jacobian(sample), compute_uv=False
        )
        condition[index] = singular_values[0] / max(singular_values[-1], 1e-8)

    print(f"采样点数：{args.samples}")
    if selected_font is None:
        print("提示：未找到中文字体，图中文字可能显示异常；请安装Noto Sans CJK。")
    else:
        print(f"绘图字体：{selected_font}")
    print("X 范围：", np.round([points[:, 0].min(), points[:, 0].max()], 3))
    print("Y 范围：", np.round([points[:, 1].min(), points[:, 1].max()], 3))
    print("Z 范围：", np.round([points[:, 2].min(), points[:, 2].max()], 3))

    fig = plt.figure(figsize=(8, 7))
    axis = fig.add_subplot(projection="3d")
    scatter = axis.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        c=np.log10(condition),
        s=5,
        cmap="viridis",
    )
    axis.set_xlabel("X / m")
    axis.set_ylabel("Y / m")
    axis.set_zlabel("Z / m")
    axis.set_title("SO-101 教学模型工作空间（颜色表示奇异性趋势）")
    fig.colorbar(scatter, label="log10(雅可比条件数)")
    if args.save:
        fig.savefig(args.save, dpi=140)
        print("图片已保存到：", args.save)
    if not args.no_show:
        plt.show()
