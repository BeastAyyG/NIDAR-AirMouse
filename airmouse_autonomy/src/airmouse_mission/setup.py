from setuptools import setup

package_name = "airmouse_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="airmouse",
    maintainer_email="dev@local",
    description=(
        "Mission monitor and failsafe FSM cores (ROS-independent) for the "
        "AirMouse autonomy stack."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = airmouse_mission.mission_node:main",
            "offboard_bridge = airmouse_mission.offboard_bridge_node:main",
        ],
    },
)