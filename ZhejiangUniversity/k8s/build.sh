#!/bin/bash

# 设置错误时退出
set -e

# 设置变量
REGISTRY="192.168.31.115:5000"
ROS1_IMAGE_NAME="ros1-robot"
ROS2_IMAGE_NAME="ros2-robot"
IMAGE_TAG="1.0"
ROS1_FULL_IMAGE_NAME="${REGISTRY}/arm/nesc/${ROS1_IMAGE_NAME}:${IMAGE_TAG}"
ROS2_FULL_IMAGE_NAME="${REGISTRY}/arm/nesc/${ROS2_IMAGE_NAME}:${IMAGE_TAG}"

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "开始构建 ROS 机器人镜像..."

# 检查 Docker 是否运行
if ! docker info &> /dev/null; then
    echo "Docker 未运行，请先启动 Docker"
    exit 1
fi

# 构建 ROS1 镜像
echo "构建 ROS1 镜像 ${ROS1_FULL_IMAGE_NAME}..."
docker build -t ${ROS1_FULL_IMAGE_NAME} \
    -f ${PROJECT_ROOT}/dockerfile/Dockerfile \
    --build-arg ROS_DISTRO=noetic \
    ${PROJECT_ROOT}

# 构建 ROS2 镜像
echo "构建 ROS2 镜像 ${ROS2_FULL_IMAGE_NAME}..."
docker build -t ${ROS2_FULL_IMAGE_NAME} \
    -f ${PROJECT_ROOT}/dockerfile/Dockerfile.ros2 \
    --build-arg ROS_DISTRO=foxy \
    ${PROJECT_ROOT}

# 推送镜像到仓库
# echo "推送镜像到仓库..."
# docker push ${ROS1_FULL_IMAGE_NAME}
# docker push ${ROS2_FULL_IMAGE_NAME}

echo "镜像构建和推送完成！" 