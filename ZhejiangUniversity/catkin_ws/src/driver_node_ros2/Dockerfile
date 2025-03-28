# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop

# 安装基础依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install paho-mqtt opencv-python numpy

# 创建工作目录
WORKDIR /ros_ws/src/driver_node_ros2

# 复制包文件（确保本地有正确的 package.xml 和 setup.py）
COPY . .

# 构建 ROS2 包
WORKDIR /ros_ws
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install --packages-select driver_node_ros2

# 设置环境变量
ENV ROS_DOMAIN_ID=0

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
source /ros_ws/install/setup.bash\n\
exec ros2 launch driver_node_ros2 deviceshifu_driver.launch.py' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# 设置入口点
ENTRYPOINT ["/entrypoint.sh"]