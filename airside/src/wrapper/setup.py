from setuptools import find_packages, setup

package_name = 'wrapper'

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
    description='Wrapper for monorepo utilities',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera = wrapper.camera_node:main',
            'map_manager = wrapper.map_manager_node:main',
            'ground_fusion = wrapper.ground_fusion_node:main',
            'sim_target = wrapper.sim_target_node:main',
            'viz = wrapper.viz_node:main',
        ],
    },
)
