from setuptools import find_packages, setup


package_name = "moonmapper_perception"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@TODO.no",
    description="Perception nodes for MoonMapper (placeholders).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "triad_serial_node = moonmapper_perception.triad_serial_node:main",
            "microscope_camera_node = moonmapper_perception.microscope_camera_node:main",
            "feature_extraction_node = moonmapper_perception.feature_extraction_node:main",
        ],
    },
)

