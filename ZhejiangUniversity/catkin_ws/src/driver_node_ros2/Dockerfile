# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop

# 安装依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install paho-mqtt opencv-python numpy

# 创建工作空间
WORKDIR /ros_ws/src
COPY . .

# 构建
WORKDIR /ros_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

# 设置启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
source /ros_ws/install/setup.bash\n\
exec ros2 launch driver_node_ros2 deviceshifu_driver.launch.py' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]