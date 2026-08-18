"""3.2 Demo：使用合成棋盘角点完成相机内参标定。"""

import argparse

import cv2
import numpy as np


def generate_observations(view_count: int, noise_px: float, seed: int = 7):
    """生成多视角棋盘2D-3D对应、图像尺寸以及相机参数真值。"""
    rng = np.random.default_rng(seed)
    columns, rows, square = 9, 6, 0.024
    object_points = np.zeros((columns * rows, 3), dtype=np.float32)
    object_points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square
    image_size = (640, 480)
    true_k = np.array([[570.0, 0.0, 318.0], [0.0, 565.0, 242.0], [0.0, 0.0, 1.0]])
    true_distortion = np.array([-0.10, 0.035, 0.001, -0.0008, 0.0])
    all_object_points, all_image_points = [], []
    for _ in range(view_count):
        rvec = rng.uniform([-0.35, -0.35, -0.25], [0.35, 0.35, 0.25])
        tvec = rng.uniform([-0.10, -0.08, 0.48], [0.08, 0.07, 0.85])
        image_points, _ = cv2.projectPoints(
            object_points, rvec, tvec, true_k, true_distortion
        )
        image_points += rng.normal(0.0, noise_px, image_points.shape)
        all_object_points.append(object_points.copy())
        all_image_points.append(image_points.astype(np.float32))
    return all_object_points, all_image_points, image_size, true_k, true_distortion


def main():
    """用合成棋盘观测估计内参与畸变，并报告重投影误差。"""
    parser = argparse.ArgumentParser(description="合成棋盘格相机标定")
    parser.add_argument("--views", type=int, default=24)
    parser.add_argument("--noise-px", type=float, default=0.18)
    args = parser.parse_args()
    object_points, image_points, image_size, true_k, true_distortion = (
        generate_observations(args.views, args.noise_px)
    )
    rms, estimated_k, estimated_distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    per_view = []
    for model, observed, rvec, tvec in zip(
        object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            model, rvec, tvec, estimated_k, estimated_distortion
        )
        per_view.append(float(np.linalg.norm(projected - observed) / np.sqrt(len(model))))

    print("真实K：\n", true_k)
    print("估计K：\n", np.round(estimated_k, 3))
    print("真实畸变：", true_distortion)
    print("估计畸变：", np.round(estimated_distortion.ravel(), 5))
    print(f"OpenCV RMS重投影误差：{rms:.4f} px")
    print(f"最差单视角误差：{max(per_view):.4f} px")
    print("提示：增大--noise-px或减少--views，观察标定稳定性变化。")


if __name__ == "__main__":
    main()
