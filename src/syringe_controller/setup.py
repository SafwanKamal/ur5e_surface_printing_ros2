from glob import glob
from setuptools import find_packages, setup

package_name = "syringe_controller"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml"),
        ),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Safwan Kamal",
    maintainer_email="maintainer@example.com",
    description="Pulse and calibrated-volume syringe serial controller",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "syringe_serial_node = "
            "syringe_controller.syringe_serial_node:main",
        ],
    },
)
