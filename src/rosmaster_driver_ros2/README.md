# ROSMASTER X3 ROS2 Driver

这是ROSMASTER X3机器人的ROS2驱动程序包。

## 目录结构

```
rosmaster_driver_ros2/
├── rosmaster_driver_ros2/          # Python包目录
│   ├── __init__.py
│   ├── rosmaster_driver_ros2.py    # 主驱动节点
│   ├── launch/                     # 启动文件目录
│   │   └── test_driver.launch.py   # 测试驱动启动文件
│   ├── test/                       # 测试代码目录
│   │   ├── __init__.py
│   │   └── test_driver.py         # 测试驱动节点
│   └── resource/                   # 资源文件目录
├── package.xml                     # 包配置文件
└── setup.py                        # Python包安装配置
```

## 功能特性

- 支持ROSMASTER X3机器人的所有基本功能
- 提供传感器数据发布（IMU、激光雷达、摄像头等）
- 支持运动控制
- 提供PID参数设置服务
- 包含测试驱动节点用于模拟测试

## 使用方法

1. 编译包：
```bash
colcon build --packages-select rosmaster_driver_ros2
```

2. 设置环境变量：
```bash
source install/setup.bash
```

3. 启动驱动节点：
```bash
ros2 launch rosmaster_driver_ros2 rosmaster_driver.launch.py
```

4. 启动测试驱动节点：
```bash
ros2 launch rosmaster_driver_ros2 test_driver.launch.py
```

## 依赖项

- ROS2 Humble或更高版本
- Python 3.8或更高版本
- OpenCV
- NumPy
- cv_bridge 