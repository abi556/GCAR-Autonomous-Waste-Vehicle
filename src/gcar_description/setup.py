import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gcar_description'

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
        # Include URDF/Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro')) + glob(os.path.join('urdf', '*.urdf'))),
        # Include RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        # Include meshes (if any)
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiy',
    maintainer_email='abiymit@outlook.com',
    description='GCAR robot URDF description with 3-DOF arm, sensors, and planar move plugin',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
