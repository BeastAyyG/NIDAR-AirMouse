from setuptools import setup

package_name = "airmouse_navigation"

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
    description="Global/local planning and drone-footprint collision checks for AirMouse.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigation_node = airmouse_navigation.navigation_node:main",
        ],
    },
)