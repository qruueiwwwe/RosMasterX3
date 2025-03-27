# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop 

# 安装构建工具（Ubuntu 下无需额外安装 bash）
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install paho-mqtt opencv-python numpy

WORKDIR /app

# 复制 ROS2 包（假设 Dockerfile 与 driver_node_ros2 同级）
COPY driver_node_ros2/ ./driver_node_ros2/

# 关键修复：使用 . 替代 source
RUN . /opt/ros/foxy/setup.bash && \
    colcon build --packages-select driver_node_ros2 --build-base /app/build --install-base /app/install

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV PATH="/app/install/driver_node_ros2/bin:$PATH"
ENV PYTHONPATH="/app/install/driver_node_ros2/lib/python3.8/site-packages:$PYTHONPATH"

CMD ["ros2", "run", "driver_node_ros2", "driver_node.py"]