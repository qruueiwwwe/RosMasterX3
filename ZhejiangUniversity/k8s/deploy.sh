#!/bin/bash

# 设置环境变量
export KUBECONFIG=~/.kube/config

# 创建命名空间
kubectl apply -f namespace.yaml

# 创建配置
kubectl apply -f configmap.yaml
kubectl apply -f secret.yaml

# 部署应用
kubectl apply -f deployment.yaml
kubectl apply -f service.yaml
kubectl apply -f network-policy.yaml

# 等待部署完成
kubectl rollout status deployment/ros-robot -n devices

# 检查服务状态
kubectl get pods -n devices -l app=ros-robot 