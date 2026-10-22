from glob import glob
from setuptools import setup

package_name = "team_interface_mocks"

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
    description="1조(Human Hand Perception)·4조(Tactile) 인터페이스 mock 과 4조 Arduino FSR 브릿지",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "mock_hand_perception_node = team_interface_mocks.mock_hand_perception_node:main",
        "mock_tactile_node = team_interface_mocks.mock_tactile_node:main",
        "tactile_serial_bridge_node = team_interface_mocks.tactile_serial_bridge_node:main",
    ]},
)
