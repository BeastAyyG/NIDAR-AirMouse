from setuptools import setup

package_name = "airmouse_validation"

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
        "Offline replay harness, ground-truth validation metrics (map F1, "
        "collisions, survivor matching) and markdown/PNG/JSON report generator."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
