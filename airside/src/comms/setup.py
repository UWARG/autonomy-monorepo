from setuptools import find_packages, setup

package_name = 'comms'

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
    description='Behaviour tree status to the ground over MAVLink STATUSTEXT',
    license='MIT',
    entry_points={
        'console_scripts': [
            'behaviour_tree_reporter = comms.behaviour_tree_reporter:main',
        ],
    },
)