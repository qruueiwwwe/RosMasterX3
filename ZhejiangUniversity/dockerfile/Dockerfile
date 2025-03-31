# 构建阶段
FROM ros:noetic-ros-base AS builder

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

# 安装构建依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-joy \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install --no-cache-dir paho-mqtt flask requests

# 创建工作目录
WORKDIR /ros_ws

# 复制 ROS1 工作空间
COPY catkin_ws/src/driver_node_ros1 /ros_ws/src/driver_node_ros1/
COPY catkin_ws/src/wheeltec_robot_rc /ros_ws/src/wheeltec_robot_rc/

# 构建 ROS1 工作空间
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin_make"

# 最终阶段
FROM ros:noetic-ros-base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

# 安装运行时依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    mosquitto \
    mosquitto-clients \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-joy \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install --no-cache-dir paho-mqtt flask requests

# 创建工作目录
WORKDIR /ros_ws

# 从构建阶段复制构建结果
COPY --from=builder /ros_ws/devel /ros_ws/devel
COPY --from=builder /ros_ws/build /ros_ws/build

# 复制配置文件
COPY dockerfile/params.yaml /ros_ws/config/
COPY dockerfile/http_server.py /ros_ws/scripts/
COPY dockerfile/entrypoint.sh /ros_ws/scripts/

# 设置环境变量
ENV PYTHONPATH=/ros_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}
ENV PATH=/ros_ws/devel/bin:/opt/ros/$ROS_DISTRO/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:/ros_ws/devel/lib:${LD_LIBRARY_PATH:-}

# 设置入口点脚本权限
RUN chmod +x /ros_ws/scripts/entrypoint.sh

# 设置入口点
ENTRYPOINT ["/ros_ws/scripts/entrypoint.sh"]
