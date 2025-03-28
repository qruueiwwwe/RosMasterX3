# 使用官方 ROS2 Foxy 镜像
FROM ubuntu:20.04

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive

# 安装必要的系统依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# 添加 ROS 2 软件源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS 2
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions \
    python3-ros2cli \
    python3-ros2pkg \
    python3-setuptools \
    python3-pkg-resources \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install paho-mqtt colcon-common-extensions setuptools==58.2.0

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
ENV ROS_DISTRO=foxy
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
ENV PYTHONPATH=/ros2_ws/install/lib/python3.10/site-packages:${PYTHONPATH:-}
ENV PATH=/ros2_ws/install/bin:/opt/ros/foxy/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib:${LD_LIBRARY_PATH:-}

# 创建启动脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
cd /ros2_ws\n\
exec "$@"' > /ros2_ws/start.sh && \
    chmod +x /ros2_ws/start.sh

# 设置入口点
ENTRYPOINT ["/ros2_ws/start.sh"]
CMD ["python3", "-c", "import rclpy; from driver_node_ros2.driver_node import main; main()"] 
