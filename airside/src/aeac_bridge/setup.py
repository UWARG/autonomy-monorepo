from setuptools import find_packages, setup

package_name = "aeac_bridge"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "websockets>=12,<16"],
    zip_safe=True,
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="Bidirectional ROS bridge for AEAC telemetry and traffic.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = aeac_bridge.node:main",
        ],
    },
)
