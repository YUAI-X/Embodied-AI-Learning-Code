"""3.2 Demo：合成Eye-in-Hand数据并求解AX=XB。"""
# 作者：宇哥的具身笔记


import argparse

import cv2
import numpy as np

from .transforms import (
    invert_transform,
    make_transform,
    rotation_error_deg,
)


def random_transform(rng, angle_scale=0.8, translation_scale=0.25):
    """生成有界随机SE(3)变换，用于构造可重复手眼标定数据。"""
    rvec = rng.normal(size=3)
    rvec = rvec / np.linalg.norm(rvec) * rng.uniform(0.15, angle_scale)
    rotation, _ = cv2.Rodrigues(rvec)
    translation = rng.uniform(-translation_scale, translation_scale, size=3)
    return make_transform(rotation, translation)


def generate_hand_eye_data(count=24, seed=11):
    """生成Eye-in-Hand位姿对，并返回T_gripper_camera真值与OpenCV输入。"""
    rng = np.random.default_rng(seed)
    # 固定未知量：相机坐标中的点如何变换到夹爪坐标，即T_gripper_camera。
    true_gripper_camera = random_transform(rng, 0.45, 0.08)
    base_target = random_transform(rng, 0.35, 0.18)
    base_target[2, 3] = 0.15
    gripper_to_base_rotations, gripper_to_base_translations = [], []
    target_to_camera_rotations, target_to_camera_translations = [], []
    for _ in range(count):
        base_gripper = random_transform(rng, 1.0, 0.25)
        base_gripper[2, 3] += 0.35
        base_camera = base_gripper @ true_gripper_camera
        camera_target = invert_transform(base_camera) @ base_target
        gripper_to_base_rotations.append(base_gripper[:3, :3])
        gripper_to_base_translations.append(base_gripper[:3, 3])
        target_to_camera_rotations.append(camera_target[:3, :3])
        target_to_camera_translations.append(camera_target[:3, 3])
    return (
        true_gripper_camera,
        gripper_to_base_rotations,
        gripper_to_base_translations,
        target_to_camera_rotations,
        target_to_camera_translations,
    )


def main():
    """求解合成Eye-in-Hand外参并报告毫米与角度误差。"""
    parser = argparse.ArgumentParser(description="Eye-in-Hand手眼标定")
    parser.add_argument("--poses", type=int, default=24)
    args = parser.parse_args()
    true_transform, r_g2b, t_g2b, r_t2c, t_t2c = generate_hand_eye_data(args.poses)
    rotation, translation = cv2.calibrateHandEye(
        r_g2b,
        t_g2b,
        r_t2c,
        t_t2c,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    estimate = make_transform(rotation, translation)
    translation_error_mm = np.linalg.norm(
        estimate[:3, 3] - true_transform[:3, 3]
    ) * 1000.0
    print("真值T_gripper_camera：\n", np.round(true_transform, 5))
    print("估计T_gripper_camera：\n", np.round(estimate, 5))
    print(f"平移误差：{translation_error_mm:.4f} mm")
    print(f"旋转误差：{rotation_error_deg(estimate, true_transform):.6f} deg")
    print("变换链：T_base_object = T_base_camera @ T_camera_object")


if __name__ == "__main__":
    main()
