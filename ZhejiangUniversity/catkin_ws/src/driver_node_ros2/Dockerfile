# 使用官方 ROS2 Foxy 镜像
FROM osrf/ros:foxy-desktop 

# 安装构建工具和依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    python3-yaml \
    libgtest-dev \
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
    colcon build --symlink-install --packages-select driver_node_ros2 \
    --packages-ignore ament_lint ament_package ament_cmake_core ament_flake8 \
    ament_cmake_export_definitions ament_cmake_export_include_directories \
    ament_cmake_export_libraries ament_cmake_export_link_flags \
    ament_cmake_include_directories ament_cmake_libraries ament_cmake_python \
    ament_cmake_version ament_pep257 ament_cmake_export_dependencies \
    ament_cmake_export_interfaces ament_cmake_export_targets \
    ament_cmake_target_dependencies ament_cmake_test ament_copyright \
    ament_cmake ament_cmake_gtest ament_cmake_pytest ament_index_python \
    domain_coordinator ament_cmake_gmock rpyutils ament_cmake_ros \
    fastrtps_cmake_module python_cmake_module rmw_implementation_cmake \
    rosidl_adapter rosidl_runtime_cpp rosidl_typesupport_interface \
    spdlog_vendor rosidl_parser tracetools rcutils rosidl_cmake \
    rcpputils rosidl_runtime_c libyaml_vendor rcl_logging_spdlog rmw \
    rosidl_generator_c rosidl_typesupport_introspection_c \
    rcl_yaml_param_parser rosidl_typesupport_fastrtps_cpp \
    rosidl_typesupport_introspection_cpp rosidl_typesupport_fastrtps_c \
    rosidl_typesupport_c rosidl_generator_py rosidl_typesupport_cpp \
    rosidl_default_runtime builtin_interfaces rmw_dds_common \
    unique_identifier_msgs action_msgs rcl_interfaces \
    rmw_fastrtps_shared_cpp rosgraph_msgs std_msgs geometry_msgs \
    rmw_fastrtps_cpp rmw_implementation sensor_msgs cv_bridge rcl \
    rcl_action rclpy

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