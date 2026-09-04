"""从安装后的 Xacro 中读取并打印 SO-101 关节信息。"""
# 作者：宇哥的具身笔记


import subprocess
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory


def main():
    """展开已安装的SO101 Xacro，并打印全部可动关节及其限制。"""
    xacro_path = (
        get_package_share_directory("so101_description") + "/urdf/so101.urdf.xacro"
    )
    urdf_text = subprocess.check_output(["xacro", xacro_path], text=True)
    root = ET.fromstring(urdf_text)

    print("\nSO-101 教学模型关节表")
    print("-" * 92)
    print(f"{'关节名':<20}{'类型':<12}{'轴':<18}{'下限(rad)':<14}{'上限(rad)':<14}")
    print("-" * 92)
    movable = 0
    for joint in root.findall("joint"):
        joint_type = joint.get("type", "")
        if joint_type == "fixed":
            continue
        movable += 1
        axis = joint.find("axis")
        limit = joint.find("limit")
        print(
            f"{joint.get('name', ''):<20}{joint_type:<12}"
            f"{(axis.get('xyz') if axis is not None else '-'):<18}"
            f"{(limit.get('lower') if limit is not None else '-'):<14}"
            f"{(limit.get('upper') if limit is not None else '-'):<14}"
        )
    print("-" * 92)
    print(f"可动关节共 {movable} 个：5 个机械臂关节 + 1 个夹爪关节。")
    print("提示：URDF 能描述结构和限制，但不能单独给出真实负载、精度和控制性能。\n")
