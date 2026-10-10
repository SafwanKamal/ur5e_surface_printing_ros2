from setuptools import find_packages, setup

setup(
    name='surface_printing_gui', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/surface_printing_gui']),
        ('share/surface_printing_gui', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Safwan Kamal', maintainer_email='maintainer@example.com',
    description='Desktop workflow console for UR5e surface printing', license='Apache-2.0',
    entry_points={'console_scripts': ['printing_gui = surface_printing_gui.app:main']},
)
