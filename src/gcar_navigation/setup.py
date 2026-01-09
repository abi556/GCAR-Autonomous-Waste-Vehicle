import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gcar_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Include params files
        (os.path.join('share', package_name, 'params'),
            glob(os.path.join('params', '*.yaml'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Include rviz configs
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiy',
    maintainer_email='abiymit@outlook.com',
    description='GCAR Navigation package with SLAM Toolbox and Nav2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_wasd = gcar_navigation.teleop_wasd:main',
        ],
    },
)
