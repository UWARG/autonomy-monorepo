from setuptools import find_packages, setup

package_name = 'ground_comms'

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
    description='Fuses sensor data into messages for the ground station',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ground_fusion = ground_comms.ground_fusion_node:main',
        ],
    },
)
