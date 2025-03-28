from setuptools import setup

package_name = "driver_node_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/deviceshifu_config.yaml"]),
        ("share/" + package_name + "/launch", ["launch/deviceshifu_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            # 确保此处指向正确的 Python 模块路径
            "driver_node = driver_node_ros2.driver_node:main"
        ],
    },
)