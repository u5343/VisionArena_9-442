#!/bin/bash

# Source Gazebo
source /usr/share/gazebo/setup.bash

# Source ROS2 workspace
cd /home/jiangping/Vision_Arena_2025
source install/setup.bash

# 创建 results 目录（如果不存在）
mkdir -p results

# 启动你的 launch 文件
ros2 launch camera_sim_pkg camera.launch.py
