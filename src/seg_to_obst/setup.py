from setuptools import find_packages, setup

package_name = "seg_to_obst"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moonlab",
    maintainer_email="sattwik21@iiserb.ac.in",
    description="TODO: Package description",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "seg_to_obst_pcl_node = seg_to_obst.seg_to_obst_pcl_node:main"
        ],
    },
)
