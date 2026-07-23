from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "baja_cloud_sim"


def data_files(directory):
    root = Path(directory)
    return [
        (str(Path("share") / package_name / path.parent), [str(path)])
        for path in root.rglob("*") if path.is_file()
    ]


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ] + data_files("launch") + data_files("config") + data_files("models") + data_files("urdf"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Baja Autonomous Team",
    maintainer_email="team@example.com",
    description="Cloud-ready dirt-road planning and control simulation",
    license="MIT",
    entry_points={
        "console_scripts": [
            "generate_scenario = baja_cloud_sim.scenario_generator:main",
            "truth_perception = baja_cloud_sim.truth_perception_node:main",
            "frenet_planner = baja_cloud_sim.frenet_planner_node:main",
            "path_follower = baja_cloud_sim.path_follower_node:main",
            "actuator_adapter = baja_cloud_sim.actuator_adapter_node:main",
            "evaluator = baja_cloud_sim.evaluator_node:main",
        ],
    },
)
