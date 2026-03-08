from setuptools import find_packages, setup
from glob import glob
import os

package_name = "moonlab_robots"


def get_data_files():
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
    ]

    # Recursively find all files in the config directory
    for root, _, files in os.walk("config"):
        if files:
            # Construct the destination path relative to 'share/moonlab_robots/'
            # This ensures config/kombai/mounts.yaml goes to share/moonlab_robots/config/kombai/mounts.yaml
            install_dir = os.path.join("share", package_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_list))

    return data_files


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=get_data_files(),  # type: ignore
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moonlab",
    maintainer_email="sattwik21@iiserb.ac.in",
    description="Robot descriptions and hardware configurations for MOON Lab",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "hound_controller = moonlab_robots.hound_controller:main",
            "kombai_controller = moonlab_robots.kombai_controller:main",
        ],
    },
)
