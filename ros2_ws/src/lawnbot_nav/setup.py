import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lawnbot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carolparedesf',
    maintainer_email='carolparedesf@fiuna.edu.py',
    description='Modos de navegacion, twist_mux y safety gate',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mode_manager = lawnbot_nav.mode_manager:main',
            'nav_stub = lawnbot_nav.nav_stub:main',
            'safety_gate = lawnbot_nav.safety_gate:main',
            'waypoint_follower = lawnbot_nav.waypoint_follower:main',
            'fake_robot = lawnbot_nav.fake_robot:main',
            'geofence_node = lawnbot_nav.geofence_node:main',
            'coverage_parallel = lawnbot_nav.coverage_parallel:main',
            'random_walk = lawnbot_nav.random_walk:main',
            'perimeter = lawnbot_nav.perimeter:main',
            'blade_controller = lawnbot_nav.blade_controller:main',
            'coverage_map = lawnbot_nav.coverage_map:main',
            'vision_stub = lawnbot_nav.vision_stub:main',
        ],
    },
)
