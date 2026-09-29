from setuptools import find_packages, setup


package_name = "aeac_telemetry"


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
    install_requires=["setuptools", "websockets>=12.0"],
    zip_safe=True,
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="Sends real vehicle telemetry directly to the AEAC competition server.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "telemetry = aeac_telemetry.aeac_telemetry_node:main",
        ],
    },
)
