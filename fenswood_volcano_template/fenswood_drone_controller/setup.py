from setuptools import setup
from glob import glob

package_name = "fenswood_drone_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",["resource/"+package_name]),
        ("share/" + package_name, ['package.xml']),
        ("share/" + package_name, glob('launch/*.launch.*'))
#        ("share/" + package_name, glob('Shapes/*.Shapes.*'))
#        ("share/" + package_name, glob('config/*Shapes'))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robert Clarke",
    maintainer_email="TODO",
    description="Example Python controller for Ardupilot at Fenswood",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = fenswood_drone_controller.controller:main",
            "controller_old_school = fenswood_drone_controller.controller_old_school:main",
            "controller_simple_class = fenswood_drone_controller.controller_simple_class:main",
            "controller_modular = fenswood_drone_controller.controller_modular:main",
            "controller_finite_state = fenswood_drone_controller.controller_finite_state:main",
            "image_processor = fenswood_drone_controller.image_processor:main",
            "failsafe = fenswood_drone_controller.failsafe:main"
        ]
    }
)
