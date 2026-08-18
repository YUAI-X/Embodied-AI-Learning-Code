import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SO-101 Course Maintainer",
    maintainer_email="maintainer@example.com",
    description="SO-101 第三章视觉算法教学样例",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_model_demo = so101_vision.camera_model_demo:main",
            "calibration_demo = so101_vision.calibration_demo:main",
            "hand_eye_demo = so101_vision.hand_eye_demo:main",
            "pointcloud_demo = so101_vision.pointcloud_demo:main",
            "segmentation_demo = so101_vision.segmentation_demo:main",
            "pnp_pose_demo = so101_vision.pnp_pose_demo:main",
            "grounded_sam_optional = so101_vision.grounded_sam_optional:main",
        ]
    },
)
