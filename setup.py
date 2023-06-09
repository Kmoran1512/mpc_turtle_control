from setuptools import setup

package_name = "mpc_turtle_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyle",
    maintainer_email="kmoran.1512@gmail.com",
    description="A 2D MPC for ROS2 turtlebot",
    license="Apache 2.0",
    entry_points={
        "console_scripts": ["mpc = mpc_turtle_control.mpc_publisher:main"],
    },
)
