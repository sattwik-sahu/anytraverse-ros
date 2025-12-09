from setuptools import find_packages, setup

package_name = "anytraverse_ros"

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
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "anytraverse_node = anytraverse_ros.anytraverse_node:main",
            "oakd_node = anytraverse_ros.oakd_node:main",
        ],
    },
)
