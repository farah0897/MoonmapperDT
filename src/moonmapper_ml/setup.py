from glob import glob
from setuptools import find_packages, setup


package_name = "moonmapper_ml"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        # Install any runtime model artifacts placed in src/moonmapper_ml/models/
        # (These files are typically ignored by git, but still useful locally.)
        (f"share/{package_name}/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@TODO.no",
    description="Runtime ML inference nodes for MoonMapper (placeholders).",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ml_inference_node = moonmapper_ml.inference_node:main",
        ],
    },
)

