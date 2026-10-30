import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    share = get_package_share_directory('lawnbot_nav')
    # config:=odom -> cuadrado de ejemplo en odom (waypoint_follower.yaml)
    # config:=map  -> rutas de coverage_parallel en map, con auto-inicio (waypoint_follower_map.yaml)
    name = {'odom': 'waypoint_follower.yaml', 'map': 'waypoint_follower_map.yaml'}[
        LaunchConfiguration('config').perform(context)]
    return [Node(package='lawnbot_nav', executable='waypoint_follower', name='waypoint_follower',
                 output='screen',
                 parameters=[os.path.join(share, 'config', name),
                             {'use_sim_time': LaunchConfiguration('use_sim_time')}])]


def generate_launch_description():
    # Se lanza junto a nav.launch.py: la salida pasa por twist_mux y safety_gate
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config', default_value='odom', choices=['odom', 'map']),
        OpaqueFunction(function=_launch),
    ])
