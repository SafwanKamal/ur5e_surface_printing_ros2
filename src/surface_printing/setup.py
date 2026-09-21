from setuptools import find_packages, setup

package_name = "surface_printing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Safwan Kamal",
    maintainer_email="maintainer@example.com",
    description="Plan and execute a surface line with syringe flow",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "make_demo_path = surface_printing.make_demo_path:main",
            "convert_surface_csv = surface_printing.convert_surface_csv:main",
            "plan_surface_path = "
            "surface_printing.plan_surface_path:main",
            "execute_surface_print = "
            "surface_printing.execute_surface_print:main",
            "hardware_preflight = surface_printing.hardware_preflight:main",
        ],
    },
)
