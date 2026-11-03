from glob import glob
from setuptools import setup

package_name = "hand_motion_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KHR",
    maintainer_email="khr.fain0516@gmail.com",
    description="3조 Motion Control: Motion Mapping, 100 Hz 하위 제어기, STS3215/PCA9685 서보 버스",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "motion_mapping_node = hand_motion_control.motion_mapping_node:main",
        "low_level_controller_node = hand_motion_control.low_level_controller_node:main",
        "servo_bus_node = hand_motion_control.servo_bus_node:main",
    ]},
)
