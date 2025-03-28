#!/bin/bash
set -e

# source ROS workspace
source /opt/ros/$ROS_DISTRO/setup.bash
source /ros_ws/devel/setup.bash
[[ -f /opt/ws_rmw_zenoh/install/setup.bash ]] && source /opt/ws_rmw_zenoh/install/setup.bash
[[ -f $WORKSPACE/devel/setup.bash ]] && source $WORKSPACE/devel/setup.bash
[[ -f $WORKSPACE/install/setup.bash ]] && source $WORKSPACE/install/setup.bash

# exec as dockeruser with configured UID/GID
if [[ $DOCKER_UID && $DOCKER_GID ]]; then
    if ! getent group $DOCKER_GID > /dev/null 2>&1; then
        groupadd -g $DOCKER_GID $DOCKER_USER
    fi
    if ! getent passwd $DOCKER_UID > /dev/null 2>&1; then
        useradd -s /bin/bash \
                -u $DOCKER_UID \
                -g $DOCKER_GID \
                --create-home \
                --home-dir /home/$DOCKER_USER \
                --groups sudo,video \
                --password "$(openssl passwd -1 $DOCKER_USER)" \
                $DOCKER_USER && \
                touch /home/$DOCKER_USER/.sudo_as_admin_successful
        cp /root/.bashrc /home/$DOCKER_USER
        ln -s $WORKSPACE /home/$DOCKER_USER/ws
        chown -h $DOCKER_UID:$DOCKER_GID $WORKSPACE /home/$DOCKER_USER/ws /home/$DOCKER_USER/.sudo_as_admin_successful
        if [[ -d $WORKSPACE/src ]]; then
            chown -R $DOCKER_USER:$DOCKER_USER $WORKSPACE/src
        fi
    fi
    [[ $(pwd) == "$WORKSPACE" ]] && cd /home/$DOCKER_USER/ws
    exec gosu $DOCKER_USER "$@"
    
else
    exec "$@"
fi
rosrun topic_tools throttle messages /imu 1.0 /imu_low &
roslaunch mqtt_client standalone.launch params_file:="/data/params.yaml"

# 启动 Mosquitto
service mosquitto start

# 启动 ROS 节点
if [ "$ROS_DISTRO" = "noetic" ]; then
    # ROS1 节点
    roslaunch driver_node_ros1 robot.launch &
else
    # ROS2 节点
    ros2 run driver_node_ros2 driver_node --ros-args -p config_file:=/ros_ws/config/params.yaml &
fi

# 启动 MQTT 桥接服务
ros2 run mqtt_bridge mqtt_bridge_node --ros-args -p config_file:=/ros_ws/config/params.yaml &

# 启动 HTTP 服务器
python3 /ros_ws/scripts/http_server.py &

# 等待所有后台进程
wait