import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawnbot_nav')
    cfg = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Cadena: bridge -> /cmd_vel_manual, stubs -> /cmd_vel_<patron>
    #         twist_mux -> /cmd_vel_mux -> safety_gate -> /cmd_vel (unico publicador) -> ESP32
    stubs = [
        Node(package='lawnbot_nav', executable='nav_stub', name=f'nav_stub_{p}',
             output='screen', parameters=[cfg('nav_stubs.yaml'), use_sim_time])
        for p in ('parallel', 'random', 'perimeter')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(package='lawnbot_nav', executable='mode_manager', name='mode_manager',
             output='screen', parameters=[cfg('mode_manager.yaml'), use_sim_time]),

        *stubs,

        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[cfg('twist_mux.yaml'), use_sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),

        Node(package='lawnbot_nav', executable='geofence_node', name='geofence_node',
             output='screen', parameters=[cfg('geofence.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='safety_gate', name='safety_gate',
             output='screen', parameters=[cfg('safety_gate.yaml'), use_sim_time]),
    ])
