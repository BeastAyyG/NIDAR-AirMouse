from setuptools import setup

package_name = "airmouse_localization"

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
        "Truth-pose DEV baseline (Gazebo ground truth) and estimated-vs-truth "
        "localization evaluator. VIO/SLAM integration is a Goal-4 runtime follow-up."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_node = airmouse_localization.localization_node:main",
        ],
    },
)
