from setuptools import find_packages, setup

package_name = 'tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WARG Autonomy Subteam',
    maintainer_email='uwarg@uwaterloo.ca',
    description='LTE tracker MQTT -> ROS 2 bridge and CSV recorder',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tracker_bridge = tracker.tracker_bridge:main',
        ],
    },
)