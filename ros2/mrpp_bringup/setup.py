from glob import glob

from setuptools import setup

package_name = "mrpp_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Peeranat",
    maintainer_email="peeranat5859@gmail.com",
    description="Shared-map localization for the multi-robot path planning project.",
    license="MIT",
)
