# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install paho-mqtt

# 创建工作空间目录
WORKDIR /ros2_ws/src/driver_node_ros2
COPY . .

# 创建正确的Python包结构
RUN mkdir -p driver_node_ros2 && \
    touch driver_node_ros2/__init__.py && \
    cp -r src/driver_node_ros2/* driver_node_ros2/ 2>/dev/null || true

# 设置工作空间
WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install"

# 设置环境变量
ENV PYTHONPATH=/ros2_ws/install/lib/python3.10/site-packages:${PYTHONPATH:-}
ENV PATH=/ros2_ws/install/bin:${PATH}

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
cd /ros2_ws\n\
python3 -c "import rclpy; from driver_node_ros2.driver_node import main; main()"' > /ros2_ws/start.sh && \
    chmod +x /ros2_ws/start.sh

# 设置入口点
ENTRYPOINT ["/ros2_ws/start.sh"] 