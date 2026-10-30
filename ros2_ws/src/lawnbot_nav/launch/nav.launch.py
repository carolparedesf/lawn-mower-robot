import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lawnbot_nav')
    cfg = lambda name: os.path.join(share, 'config', name)  # noqa: E731
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    # Cadena: bridge -> /cmd_vel_manual, stubs -> /cmd_vel_<patron>
    #         twist_mux -> /cmd_vel_mux -> safety_gate -> /cmd_vel (unico publicador) -> ESP32
    # Los tres modos automaticos tienen nodo real (coverage_parallel + waypoint_follower, random_walk,
    # perimeter). Los stubs a ciegas quedan apagados salvo use_<modo>_stub:=true; encenderlos junto
    # al nodo real daria dos publicadores en el mismo /cmd_vel_<algoritmo>.
    stubs = [
        Node(package='lawnbot_nav', executable='nav_stub', name=f'nav_stub_{p}',
             output='screen', parameters=[cfg('nav_stubs.yaml'), use_sim_time],
             condition=IfCondition(LaunchConfiguration(f'use_{p}_stub')))
        for p in ('parallel', 'random', 'perimeter')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_parallel_stub', default_value='false'),
        DeclareLaunchArgument('use_random_stub', default_value='false'),
        DeclareLaunchArgument('use_perimeter_stub', default_value='false'),
        # Camara sin calibrar: vision_stub publica obstaculos vacios y grass_present=false (la cuchilla no enciende)
        DeclareLaunchArgument('use_vision_stub', default_value='false'),
        # PELIGRO: con true el stub afirma que hay pasto sin ninguna deteccion. Solo para pruebas de banco con los
        # motores de corte desconectados. Apagado por defecto; mientras este activo el log lo advierte cada 10 s.
        DeclareLaunchArgument('vision_stub_grass_present', default_value='false'),

        Node(package='lawnbot_nav', executable='mode_manager', name='mode_manager',
             output='screen', parameters=[cfg('mode_manager.yaml'), use_sim_time]),

        *stubs,

        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[cfg('twist_mux.yaml'), use_sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),

        Node(package='lawnbot_nav', executable='geofence_node', name='geofence_node',
             output='screen', parameters=[cfg('geofence.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='coverage_parallel', name='coverage_parallel',
             output='screen', parameters=[cfg('coverage.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='random_walk', name='random_walk',
             output='screen', parameters=[cfg('random_walk.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='perimeter', name='perimeter',
             output='screen', parameters=[cfg('perimeter.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='coverage_map', name='coverage_map',
             output='screen', parameters=[cfg('coverage_map.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='blade_controller', name='blade_controller',
             output='screen', parameters=[cfg('blade_controller.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='robot_events', name='robot_events',
             output='screen', parameters=[cfg('robot_events.yaml'), use_sim_time]),

        Node(package='lawnbot_nav', executable='vision_stub', name='vision_stub', output='screen',
             parameters=[{'grass_present': ParameterValue(LaunchConfiguration('vision_stub_grass_present'),
                                                          value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('use_vision_stub'))),

        Node(package='lawnbot_nav', executable='safety_gate', name='safety_gate',
             output='screen', parameters=[cfg('safety_gate.yaml'), use_sim_time]),
    ])
