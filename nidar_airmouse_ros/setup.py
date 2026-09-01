from setuptools import find_packages, setup


package_name = "nidar_airmouse_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/airmouse_sim.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/airmouse.yaml", "config/airmouse_compete.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="PX4/Gazebo simulation utilities for the NIDAR AirMouse project.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission-manager = nidar_airmouse_sim.mission_manager:main",
            "survivor-localizer = nidar_airmouse_sim.survivor_localizer:main",
            "gcs-status = nidar_airmouse_sim.gcs_status:main",
            "sim-smoke = nidar_airmouse_sim.sim_smoke:main",
            "sim-survivor-detector = nidar_airmouse_sim.sim_survivor_detector:main",
            "lidar-safety = nidar_airmouse_sim.lidar_safety:main",
        ],
    },
)
