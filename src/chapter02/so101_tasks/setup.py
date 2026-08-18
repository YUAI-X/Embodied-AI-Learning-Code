import os
from glob import glob
from setuptools import find_packages, setup

package_name = "so101_tasks"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="SO-101 Course Maintainer",
    maintainer_email="maintainer@example.com",
    description="SO-101 MoveIt 2 教学任务",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "move_named_pose = so101_tasks.move_named_pose:main",
            "virtual_pick_place = so101_tasks.virtual_pick_place:main",
        ]
    },
)
