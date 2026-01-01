import os
from glob import glob
from setuptools import find_packages, setup

package_name = "slam_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kai Nakamura",
    maintainer_email="kaihnakamura@gmail.com",
    description="An autonomous mapping robot using SLAM (Simultaneous Localization and Mapping)",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "frontier_server = slam_robot.frontier_server:main",
            "frontier_explorer = slam_robot.frontier_explorer:main",
            "frontier_visualizer = slam_robot.frontier_visualizer:main",
        ],
    },
)
