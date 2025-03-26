# 使用 ROS Noetic core 镜像作为基础镜像
FROM ghcr.io/ika-rwth-aachen/mqtt_client:ros

# 安装Python依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 创建工作目录
WORKDIR /data

# 将 参数文件 代码复制到容器中
COPY params.yaml .
COPY deviceshifu.yaml .

# 复制自定义入口点脚本
COPY entrypoint.sh .

# 安装DeviceShifu依赖
RUN pip3 install flask requests

# 添加入口点脚本的执行权限
RUN chmod +x ./entrypoint.sh

# 设置入口点脚本
ENTRYPOINT ["./entrypoint.sh"]
#加了这个，就不会运行默认的程序
# CMD ["bash"]
