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

echo "开始部署 ROS 机器人服务..."

# 检查 Docker 是否运行
if ! docker info &> /dev/null; then
    echo "Docker 未运行，请先启动 Docker"
    exit 1
fi

# 构建镜像
echo "构建 ROS1 镜像 ${ROS1_FULL_IMAGE_NAME}..."
docker build -t ${ROS1_FULL_IMAGE_NAME} -f ../dockerfile/Dockerfile ..

echo "构建 ROS2 镜像 ${ROS2_FULL_IMAGE_NAME}..."
docker build -t ${ROS2_FULL_IMAGE_NAME} -f ../dockerfile/Dockerfile.ros2 ..

# 推送镜像到仓库
echo "推送镜像到仓库..."
docker push ${ROS1_FULL_IMAGE_NAME}
docker push ${ROS2_FULL_IMAGE_NAME}

# 设置环境变量
export KUBECONFIG=~/.kube/config

# 创建命名空间
echo "创建命名空间..."
kubectl apply -f namespace.yaml --validate=false

# 创建配置
echo "创建配置..."
kubectl apply -f configmap.yaml --validate=false
kubectl apply -f secret.yaml --validate=false

# 部署应用
echo "部署 ROS1 应用..."
kubectl apply -f deployment-ros1.yaml --validate=false
kubectl apply -f service-ros1.yaml --validate=false

echo "部署 ROS2 应用..."
kubectl apply -f deployment-ros2.yaml --validate=false
kubectl apply -f service-ros2.yaml --validate=false

# 应用网络策略
echo "应用网络策略..."
kubectl apply -f network-policy.yaml --validate=false

# 等待部署完成
echo "等待部署完成..."
kubectl rollout status deployment/ros1-robot -n devices
kubectl rollout status deployment/ros2-robot -n devices

# 检查服务状态
echo "检查服务状态..."
kubectl get pods -n devices -l app=ros1-robot
kubectl get pods -n devices -l app=ros2-robot

echo "部署完成！" 