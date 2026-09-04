from setuptools import find_packages, setup
# 作者：宇哥的具身笔记


package_name = "so101_math"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SO-101 Course Maintainer",
    maintainer_email="maintainer@example.com",
    description="SO-101 机器人学数学教学样例",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspect_so101_model = so101_math.inspect_so101_model:main",
            "pid_parameter_tuning = so101_math.pid_parameter_tuning:main",
            "safe_joint_action = so101_math.safe_joint_action:main",
            "pose_representation_demo = so101_math.pose_representation_demo:main",
            "planar_2r_fk_ik = so101_math.planar_2r_fk_ik:main",
            "so101_forward_kinematics = so101_math.so101_forward_kinematics:main",
            "so101_numerical_ik = so101_math.so101_numerical_ik:main",
            "sample_workspace = so101_math.sample_workspace:main",
        ]
    },
)
