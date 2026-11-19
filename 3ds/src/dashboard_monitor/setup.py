from setuptools import find_packages, setup

setup(
    name="dashboard_monitor", version="0.1.0", packages=find_packages(),
    data_files=[("share/ament_index/resource_index/packages", ["resource/dashboard_monitor"]),
                ("share/dashboard_monitor", ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="WARG Autonomy Subteam", maintainer_email="uwarg@uwaterloo.ca",
    description="Read-only companion CPU/GPU temperature telemetry.", license="MIT",
    entry_points={"console_scripts": [
        "temperatures = dashboard_monitor.temperature_node:main",
    ]},
)
