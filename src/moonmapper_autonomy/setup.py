import os
from glob import glob

from setuptools import find_packages, setup

package_name = "moonmapper_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoonMapper Team",
    maintainer_email="user@example.com",
    description="Simple autonomy behaviors for MoonMapper.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_obstacle_node = moonmapper_autonomy.safety_obstacle_node:main",
            "reactive_avoidance_node = moonmapper_autonomy.reactive_avoidance_node:main",
            "depth_to_scan_node = moonmapper_autonomy.depth_to_scan_node:main",
            "coverage_logger_node = moonmapper_autonomy.coverage_logger_node:main",
            "simple_goal_follower_node = moonmapper_autonomy.simple_goal_follower_node:main",
            "goal_obstacle_avoidance_node = moonmapper_autonomy.goal_obstacle_avoidance_node:main",
        ],
    },
)
