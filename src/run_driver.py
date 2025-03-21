#!/usr/bin/env python3
import os
import sys

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# 设置环境变量
os.environ['ROBOT_TEST_MODE'] = 'true'

# 导入并运行驱动程序
from rosmaster_driver_ros2 import main

if __name__ == '__main__':
    main() 