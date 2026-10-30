import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lawnbot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carolparedesf',
    maintainer_email='carolparedesf@fiuna.edu.py',
    description='Nodos de hardware: TF-Mini Plus y motores de corte',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tfmini_node = lawnbot_hardware.tfmini_node:main',
            'cutting_motors_node = lawnbot_hardware.cutting_motors_node:main',
        ],
    },
)
