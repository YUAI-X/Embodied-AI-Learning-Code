import os
from glob import glob
from setuptools import find_packages, setup

package_name = "so101_ros2_basics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SO-101 Course Maintainer",
    maintainer_email="maintainer@example.com",
    description="SO-101 ROS 2 基础教学节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_joints_server = so101_ros2_basics.move_joints_server:main",
            "move_joints_client = so101_ros2_basics.move_joints_client:main",
            "pid_parameter_node = so101_ros2_basics.pid_parameter_node:main",
            "mock_object_publisher = so101_ros2_basics.mock_object_publisher:main",
            "pose_transformer = so101_ros2_basics.pose_transformer:main",
            "rgbd_camera_publisher = so101_ros2_basics.rgbd_camera_publisher:main",
            "rgbd_sync_node = so101_ros2_basics.rgbd_sync_node:main",
            "depth_pixel_to_3d = so101_ros2_basics.depth_pixel_to_3d:main",
            "qos_mismatch_subscriber = so101_ros2_basics.qos_mismatch_subscriber:main",
        ]
    },
)
