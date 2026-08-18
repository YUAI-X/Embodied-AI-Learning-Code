import os
from glob import glob

from setuptools import find_packages, setup

package_name = "so101_vision_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SO-101 Course Maintainer",
    maintainer_email="maintainer@example.com",
    description="SO-101 第三章视觉ROS 2教学节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "synthetic_rgbd_publisher = so101_vision_ros.synthetic_rgbd_publisher:main",
            "rgbd_pointcloud_node = so101_vision_ros.rgbd_pointcloud_node:main",
            "target_localizer_node = so101_vision_ros.target_localizer_node:main",
        ]
    },
)
