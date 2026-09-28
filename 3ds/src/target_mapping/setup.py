from setuptools import find_packages, setup

package_name = 'target_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WARG Autonomy Subteam',
    maintainer_email='uwarg@uwaterloo.ca',
    description='Target logging and building-relative target localization nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'building_target_localizer = '
            'target_mapping.building_target_localizer_node:main',
            'map_manager = target_mapping.map_manager_node:main',
        ],
    },
)
