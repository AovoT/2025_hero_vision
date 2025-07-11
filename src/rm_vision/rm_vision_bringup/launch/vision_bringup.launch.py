import os
import sys

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer

# 加入 rm_vision_bringup 模块路径
from ament_index_python.packages import get_package_share_directory as gpsd
sys.path.append(os.path.join(gpsd('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    from node_desc import detector_node, tracker_node, serial_node, rsp_component

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            rsp_component,
            detector_node,
            tracker_node,
            serial_node,
        ],
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        container
    ])