from setuptools import find_packages, setup

package_name = 'moonmapper_webots'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/webots_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/moonmapper_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/moonmapper_rover_webots.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MoonMapper Team',
    maintainer_email='user@example.com',
    description='Webots world and MoonMapper rover driver for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
