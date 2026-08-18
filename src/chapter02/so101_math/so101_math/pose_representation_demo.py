"""演示旋转矩阵、齐次变换、欧拉角和四元数之间的关系。"""

import numpy as np
from .kinematics import axis_angle_matrix, translation_matrix


def rotation_to_quaternion(rotation):
    """将旋转矩阵转换为 [x, y, z, w] 四元数。"""
    trace = np.trace(rotation)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        # 教学脚本覆盖常见分支；选取最大对角元素可提高数值稳定性。
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            x, y, z, w = (
                0.25 * s,
                (rotation[0, 1] + rotation[1, 0]) / s,
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[2, 1] - rotation[1, 2]) / s,
            )
        elif index == 1:
            s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            x, y, z, w = (
                (rotation[0, 1] + rotation[1, 0]) / s,
                0.25 * s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
            )
        else:
            s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            x, y, z, w = (
                (rotation[0, 2] + rotation[2, 0]) / s,
                (rotation[1, 2] + rotation[2, 1]) / s,
                0.25 * s,
                (rotation[1, 0] - rotation[0, 1]) / s,
            )
    quaternion = np.array([x, y, z, w])
    return quaternion / np.linalg.norm(quaternion)


def main():
    """演示齐次变换组合、求逆、四元数转换和旋转不可交换性。"""
    # 假设相机相对机器人基座平移，并绕 Z 轴旋转 30°。
    base_to_camera = translation_matrix([0.18, -0.10, 0.25])
    base_to_camera = base_to_camera @ axis_angle_matrix([0, 0, 1], np.deg2rad(30))
    camera_to_object = translation_matrix([0.0, 0.0, 0.40])
    base_to_object = base_to_camera @ camera_to_object

    rotation = base_to_camera[:3, :3]
    print("base 到 camera 的齐次变换：\n", np.round(base_to_camera, 4))
    print("\n旋转矩阵逆是否等于转置：", np.allclose(np.linalg.inv(rotation), rotation.T))
    print(
        "T × T^-1 是否等于单位阵：",
        np.allclose(base_to_camera @ np.linalg.inv(base_to_camera), np.eye(4)),
    )
    print("相机中物体 [0, 0, 0.4] 转到 base 后的位置：", np.round(base_to_object[:3, 3], 4))
    print("对应四元数 [x, y, z, w]：", np.round(rotation_to_quaternion(rotation), 4))

    # 旋转矩阵乘法不满足交换律。
    rotate_x = axis_angle_matrix([1, 0, 0], np.deg2rad(45))
    rotate_y = axis_angle_matrix([0, 1, 0], np.deg2rad(30))
    print("不同旋转顺序是否相同：", np.allclose(rotate_x @ rotate_y, rotate_y @ rotate_x))
