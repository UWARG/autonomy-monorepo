from setuptools import find_packages, setup


package_name = "navigation"


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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="Position control between the engine and MAVROS.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "position_controller = navigation.position_controller_node:main",
        ],
    },
)
