from setuptools import find_packages, setup

package_name = "airside_comms"

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
    install_requires=["setuptools", "websockets"],
    zip_safe=True,
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="WebSocket bridge between the airside ROS2 system and the IMS ground station.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "airside_comms = airside_comms.node:main",
        ],
    },
)

