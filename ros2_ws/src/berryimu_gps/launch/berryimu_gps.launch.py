from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('berryimu_gps'),
        'config',
        'imu_calibration.yaml'
    )

    imu_node = Node(
        package='berryimu_gps',
        executable='imu_node',
        name='berryimu_driver',
        output='screen',
        parameters=[config],
    )

    gps_node = Node(
        package='berryimu_gps',
        executable='gps_node',
        name='berryimu_gps_node',
        output='screen',
    )

    return LaunchDescription([imu_node, gps_node])
