from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_autonomous_pkg'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.dbc')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Baja Autonomous Team',
    maintainer_email='team@example.com',
    description='VCU CAN bridge for Baja autonomous vehicle',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'can_bridge_node = car_autonomous_pkg.can_bridge_node:main',
        ],
    },
)
