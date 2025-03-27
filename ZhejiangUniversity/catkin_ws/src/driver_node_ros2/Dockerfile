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
COPY launch/driver_node.launch.py ./launch/

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:${PYTHONPATH:-}

# 设置入口点
ENTRYPOINT ["ros2", "launch", "driver_node_ros2", "driver_node.launch.py"]