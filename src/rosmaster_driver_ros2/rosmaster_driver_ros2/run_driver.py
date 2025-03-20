#!/usr/bin/env python3
import os
import sys

# 添加工作空间路径
workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(workspace_path)

# 设置环境变量
os.environ['ROBOT_TEST_MODE'] = 'true'

# 导入并运行驱动程序
from rosmaster_driver_ros2.rosmaster_driver_ros2.rosmaster_driver_ros2 import main

if __name__ == '__main__':
    main() 