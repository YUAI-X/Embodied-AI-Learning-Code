"""3.5 Demo：RANSAC+PnP、评价指标和抓取坐标链。"""
# 作者：宇哥的具身笔记


import argparse

import numpy as np

from .camera_geometry import project_points
from .pose_estimation import (
    add_error,
    adds_error,
    cube_model,
    make_demo_pose,
    solve_pose_pnp,
)
from .synthetic_data import default_intrinsics
from .transforms import make_transform, rotation_error_deg, transform_points


def main():
    """运行带外点的PnP，并演示Camera→Base→Grasp位姿链。"""
    parser = argparse.ArgumentParser(description="传统6D物体位姿估计")
    parser.add_argument("--noise-px", type=float, default=0.35)
    args = parser.parse_args()
    rng = np.random.default_rng(21)
    intrinsics = default_intrinsics(640, 480)
    model = cube_model(0.06)
    truth = make_demo_pose()
    image_points = project_points(transform_points(truth, model), intrinsics)
    image_points += rng.normal(0.0, args.noise_px, image_points.shape)
    # 故意污染一个对应点，展示RANSAC的作用。
    image_points[0] += [14.0, -10.0]
    result = solve_pose_pnp(model, image_points, intrinsics, reprojection_threshold=3.0)
    if not result.success:
        raise SystemExit("PnP求解失败")

    base_camera = make_transform(np.eye(3), [0.20, 0.0, 0.35])
    object_grasp = make_transform(np.eye(3), [0.0, 0.0, 0.08])
    base_object = base_camera @ result.transform_camera_object
    base_grasp = base_object @ object_grasp
    translation_error = np.linalg.norm(
        result.transform_camera_object[:3, 3] - truth[:3, 3]
    )

    print("PnP内点索引：", result.inliers)
    print(f"重投影RMSE：{result.reprojection_rmse:.3f} px")
    print(f"平移误差：{translation_error * 1000.0:.3f} mm")
    print(f"旋转误差：{rotation_error_deg(result.transform_camera_object, truth):.3f} deg")
    print(f"ADD：{add_error(model, result.transform_camera_object, truth) * 1000:.3f} mm")
    print(f"ADD-S：{adds_error(model, result.transform_camera_object, truth) * 1000:.3f} mm")
    print("T_base_object：\n", np.round(base_object, 4))
    print("T_base_grasp：\n", np.round(base_grasp, 4))


if __name__ == "__main__":
    main()
