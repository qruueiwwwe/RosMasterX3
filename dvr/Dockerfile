# 使用 ROS 基础镜像
FROM ros:noetic-ros-base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive

# 安装必要的依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3 \
    python3-yaml \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖
RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple paho-mqtt flask

# 创建工作目录
WORKDIR /ros_ws

# 复制驱动节点代码
COPY . /ros_ws/

# 设置执行权限
RUN chmod +x /ros_ws/driver_node.py

# 设置环境变量
ENV PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH:-}
ENV PATH=/opt/ros/noetic/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}

# 设置入口点
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && python3 /ros_ws/driver_node.py"] 