from setuptools import find_packages, setup


package_name = "engine"


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
        (f"share/{package_name}/launch", ["launch/engine.launch.py"]),
        (f"share/{package_name}/config", ["config/waypoints.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="WARG Autonomy Subteam",
    maintainer_email="uwarg@uwaterloo.ca",
    description="Behavior tree manager for airside autonomy projects.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manager = engine.manager:main",
        ],
    },
)
