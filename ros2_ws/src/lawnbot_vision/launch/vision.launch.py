import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawnbot_vision')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # No lanzar a la vez que vision_stub (use_vision_stub:=true de nav.launch.py): publican los mismos topics
        Node(package='lawnbot_vision', executable='vision_node', name='vision_node', output='screen',
             parameters=[os.path.join(share, 'config', 'vision.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
