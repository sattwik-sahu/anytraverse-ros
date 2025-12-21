from setuptools import find_packages, setup
from glob import glob
import os

package_name = "moonlab_robots"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moonlab",
    maintainer_email="sattwik21@iiserb.ac.in",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["hound_controller = moonlab_robots.hound_controller:main"],
    },
)
