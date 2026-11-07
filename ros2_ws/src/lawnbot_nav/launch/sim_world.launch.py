import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = ('room', 'field')


def world_config(share, world):
    """Ruta del YAML del mundo simulado (config/sim_<world>.yaml). ValueError si el mundo no existe."""
    if world not in WORLDS:
        raise ValueError(f"world '{world}' invalido: use {' o '.join(WORLDS)}")
    return os.path.join(share, 'config', f'sim_{world}.yaml')


def launch_fake_robot(context):
    share = get_package_share_directory('lawnbot_nav')
    config = world_config(share, LaunchConfiguration('world').perform(context))
    return [Node(package='lawnbot_nav', executable='fake_robot', name='fake_robot', output='screen',
                 parameters=[config])]


def generate_launch_description():
    # Robot simulado con sonares y bumpers en un mundo con paredes y obstaculos:
    #   room  (por defecto): cuarto de 10 x 8 m, config/sim_room.yaml
    #   field: campo abierto de 30 x 30 m con un borde lejano, config/sim_field.yaml
    # Se usa junto a nav.launch.py. NO correr ekf_local ni la odometria real: habria dos fuentes del TF.
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='room',
                              description="mundo simulado: 'room' (cuarto 10x8 m) o 'field' (campo 30x30 m)"),
        OpaqueFunction(function=launch_fake_robot),
    ])
