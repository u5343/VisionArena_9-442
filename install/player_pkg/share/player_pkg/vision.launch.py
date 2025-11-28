import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'player_pkg' 
    pkg_dir = get_package_share_directory(pkg_name)

    # 2. 定义参数文件路径 (如果需要)
    # 如果您决定以后添加参数文件，可以使用以下代码：
    # params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    # 3. 定义视觉处理节点 (vision_node)
    vision_node = Node(
        package=pkg_name,
        executable='yolo_detector_node', # <--- 您的 C++ 可执行文件名
        name='LocateNode',             # <--- ROS 2 节点名
        output='screen',
        # parameters=[params_file],      # <--- 如果使用参数文件，取消注释
    )

    # 4. 返回 Launch 描述
    return LaunchDescription([
        vision_node,

    ])
