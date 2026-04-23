from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'moonmapper_control'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MoonMapper Team',
    maintainer_email='user@example.com',
    description='Teleop and cmd_vel control for MoonMapper rover',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = moonmapper_control.teleop_keyboard:main',
        ],
    },
)
