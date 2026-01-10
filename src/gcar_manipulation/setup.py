from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gcar_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiy',
    maintainer_email='abiymit@outlook.com',
    description='Arm manipulation control for GCAR robot with preset pick and place poses',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_controller = gcar_manipulation.arm_controller:main',
            'gazebo_manager = gcar_manipulation.gazebo_manager:main',
            'pickup_coordinator = gcar_manipulation.pickup_coordinator:main',
        ],
    },
)
