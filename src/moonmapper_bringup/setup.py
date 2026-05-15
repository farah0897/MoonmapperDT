from setuptools import find_packages, setup

package_name = "moonmapper_bringup"

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(
    ("share/" + package_name + "/launch", [
        "launch/sim.launch.py",
        "launch/sensor_bringup.launch.py",
        "launch/sim_rover_clean.launch.py",
        "launch/realsense_d435.launch.py",
        "launch/camera_static_tf.launch.py",
        "launch/real_robot_navigation.launch.py",
        "launch/realsense_d435.launch.py",
        "launch/camera_static_tf.launch.py",
    ]),
)
data_files.append(
    ("share/" + package_name + "/config", [
        "config/moonmapper_rviz.rviz",
    ]),
)
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MoonMapper Team",
    maintainer_email="user@example.com",
    description="Launch files for MoonMapper rover simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_aliases = moonmapper_bringup.camera_aliases:main",
            "cmd_vel_odom_relay = moonmapper_bringup.cmd_vel_odom_relay:main",
        ],
    },
)
