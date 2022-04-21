from setuptools import setup

package_name = "perception_benchmark_tool"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kaan Colak",
    maintainer_email="kcolak@leodrive.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_benchmark_node = perception_benchmark_tool.perception_benchmark_node:main",
            "tracking_evaluation_node = perception_benchmark_tool.tracking_evaluator_node:main",
        ],
    },
)
