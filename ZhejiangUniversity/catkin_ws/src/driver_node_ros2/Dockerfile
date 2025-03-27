# 使用ROS2 Humble作为基础镜像
FROM ros:humble

# 安装系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install paho-mqtt opencv-python numpy

# 创建工作目录
WORKDIR /opt/ros/humble/share/driver_node_ros2

# 复制驱动代码和配置文件
COPY driver_node_ros2/driver_node_ros2/driver_node.py ./driver_node_ros2/
COPY driver_node_ros2/config/deviceshifu_config.yaml ./config/
COPY driver_node_ros2/launch/driver_node.launch.py ./launch/

# 设置环境变量
ENV ROS_DOMAIN_ID=0
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

# 设置入口点
ENTRYPOINT ["ros2", "launch", "driver_node_ros2", "driver_node.launch.py"]
