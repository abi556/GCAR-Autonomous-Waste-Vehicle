from setuptools import find_packages, setup

package_name = 'gcar_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiy',
    maintainer_email='abiymit@outlook.com',
    description='Safety monitoring and boundary enforcement for GCAR robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'boundary_monitor = gcar_safety.boundary_monitor:main'
        ],
    },
)
