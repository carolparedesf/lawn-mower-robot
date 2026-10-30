from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Puertos fijos por regla udev (docs/setup_usb.md): /dev/tfmini
    return LaunchDescription([
        DeclareLaunchArgument('tfmini_port', default_value='/dev/tfmini'),
        DeclareLaunchArgument('tfmini_frame_id', default_value='tf_mini_link'),

        Node(
            package='lawnbot_hardware',
            executable='tfmini_node',
            name='tfmini_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('tfmini_port'),
                'frame_id': LaunchConfiguration('tfmini_frame_id'),
            }],
        ),

        # Requiere pigpiod activo: sudo systemctl status pigpiod
        Node(
            package='lawnbot_hardware',
            executable='cutting_motors_node',
            name='cutting_motors',
            output='screen',
        ),
    ])
