from glob import glob
import os

from setuptools import setup

package_name = "perception_benchmark_tool"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kaan Colak",
    maintainer_email="kcolak@leodrive.ai",
    description="This package benchmark Autoware perception stack on Waymo dataset",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "benchmark_node = perception_benchmark_tool.benchmark_node:main",
            "waymo_player_node = perception_benchmark_tool.waymo_player_node:main",
            "autoware_workflow_runner_node = perception_benchmark_tool.autoware_workflow_runner_node:main",
        ],
    },
)
