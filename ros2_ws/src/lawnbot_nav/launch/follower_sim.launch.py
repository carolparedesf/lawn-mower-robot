import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawnbot_nav')
    cfg = os.path.join(share, 'config', 'waypoint_follower.yaml')

    # Prueba en seco: robot simulado + seguidor. NO correr ekf_local a la vez (mismo TF).
    # cmd_topic: cmd_vel_waypoint (directo) o cmd_vel (con nav.launch.py: mux + safety_gate)
    return LaunchDescription([
        DeclareLaunchArgument('cmd_topic', default_value='cmd_vel_waypoint'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('wheel_deadzone', default_value='0.0'),

        Node(package='lawnbot_nav', executable='fake_robot', name='fake_robot', output='screen',
             parameters=[{'cmd_topic': LaunchConfiguration('cmd_topic'),
                          'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'yaw': LaunchConfiguration('yaw'),
                          'wheel_deadzone': LaunchConfiguration('wheel_deadzone')}]),

        Node(package='lawnbot_nav', executable='waypoint_follower', name='waypoint_follower',
             output='screen',
             parameters=[cfg, {'require_active_mode': False}]),
    ])
