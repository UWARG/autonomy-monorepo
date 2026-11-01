from setuptools import find_packages, setup


package_name = "comms"


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
    install_requires=["setuptools", "websocket-client"],
    zip_safe=True,
    tests_require=["pytest"],
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="Comm links.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "traffic_listener = comms.traffic_listener_node:main",
            "telemetry_sender = comms.telemetry_sender_node:main",
        ],
    },
)
