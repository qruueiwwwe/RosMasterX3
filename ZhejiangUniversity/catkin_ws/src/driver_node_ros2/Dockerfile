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
WORKDIR /opt/ros/foxy/src/driver_node_ros2

# 复制整个包
COPY . .

# 构建和安装包
RUN . /opt/ros/foxy/setup.sh && \
    cd /opt/ros/foxy && \
    colcon build --symlink-install --packages-select driver_node_ros2

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
source /opt/ros/foxy/install/setup.bash\n\
exec ros2 launch driver_node_ros2 deviceshifu_driver.launch.py' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]