# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop 

# 安装构建工具和依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install paho-mqtt opencv-python numpy

# 创建工作目录
WORKDIR /opt/ros/foxy/share/driver_node_ros2

# 复制驱动代码和配置文件
COPY driver_node_ros2/driver_node.py ./driver_node_ros2/
COPY config/deviceshifu_config.yaml ./config/
COPY launch/deviceshifu_driver.launch.py ./launch/

# 创建package.xml
RUN echo '<?xml version="1.0"?>\n\
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>\n\
<package format="3">\n\
  <name>driver_node_ros2</name>\n\
  <version>0.0.1</version>\n\
  <description>DeviceShifu driver node for ROS2</description>\n\
  <maintainer email="your-email@example.com">Your Name</maintainer>\n\
  <license>TODO</license>\n\
  <depend>rclpy</depend>\n\
  <depend>geometry_msgs</depend>\n\
  <depend>sensor_msgs</depend>\n\
  <depend>std_msgs</depend>\n\
  <depend>cv_bridge</depend>\n\
  <depend>python3-opencv</depend>\n\
  <depend>python3-numpy</depend>\n\
  <depend>python3-yaml</depend>\n\
  <export>\n\
    <build_type>ament_python</build_type>\n\
  </export>\n\
</package>' > package.xml

# 创建setup.py
RUN echo 'from setuptools import setup\n\
\n\
package_name = "driver_node_ros2"\n\
\n\
setup(\n\
    name=package_name,\n\
    version="0.0.1",\n\
    packages=[package_name],\n\
    data_files=[\n\
        ("share/ament_index/resource_index/packages",\n\
            ["resource/" + package_name]),\n\
        ("share/" + package_name, ["package.xml"]),\n\
        ("share/" + package_name + "/config", ["config/deviceshifu_config.yaml"]),\n\
        ("share/" + package_name + "/launch", ["launch/deviceshifu_driver.launch.py"]),\n\
    ],\n\
    install_requires=["setuptools"],\n\
    zip_safe=True,\n\
    maintainer="Your Name",\n\
    maintainer_email="your-email@example.com",\n\
    description="DeviceShifu driver node for ROS2",\n\
    license="TODO",\n\
    tests_require=["pytest"],\n\
    entry_points={\n\
        "console_scripts": [\n\
            "driver_node = driver_node_ros2.driver_node:main"\n\
        ],\n\
    },\n\
)' > setup.py

# 创建resource目录
RUN mkdir -p resource && \
    echo "driver_node_ros2" > resource/driver_node_ros2

# 安装包
RUN . /opt/ros/foxy/setup.sh && \
    pip3 install -e .

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
exec ros2 launch driver_node_ros2 deviceshifu_driver.launch.py' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]