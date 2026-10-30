import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawnbot_nav')
    # Robot simulado en un cuarto con paredes y obstaculos (config/sim_room.yaml), con sonares y bumpers.
    # Se usa junto a nav.launch.py. NO correr ekf_local ni la odometria real: habria dos fuentes del TF.
    return LaunchDescription([
        Node(package='lawnbot_nav', executable='fake_robot', name='fake_robot', output='screen',
             parameters=[os.path.join(share, 'config', 'sim_room.yaml')]),
    ])
