from glob import glob

from setuptools import setup

package_name = "siyi_gimbal_adapter"

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
    maintainer="Mohamed Abdelkader",
    maintainer_email="mohamedashraf123@gmail.com",
    description="Course gimbal topics <-> siyi_ros2 driver.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "siyi_gimbal_adapter = siyi_gimbal_adapter.adapter_node:main",
        ],
    },
)
