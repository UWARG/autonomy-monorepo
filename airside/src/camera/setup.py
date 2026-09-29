from setuptools import find_packages, setup


package_name = "camera"


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
    description="Camera drivers and triggered image capture.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera = camera_ros.camera_node:main",
            "downward_camera = camera_ros.downward_camera_node:main",
            "triggered_image_publisher = "
            "camera_ros.triggered_image_publisher_node:main",
        ],
    },
)
