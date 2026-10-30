import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawnbot_nav')
    cfg = os.path.join(share, 'config', 'waypoint_follower.yaml')

    # Se lanza junto a nav.launch.py: la salida pasa por twist_mux y safety_gate
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lawnbot_nav', executable='waypoint_follower', name='waypoint_follower',
             output='screen',
             parameters=[cfg, {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
