#!/usr/bin/env python3
"""Reemplazo de la camara mientras no este calibrada: publica /vision/obstacles vacio y /vision/grass_present fijo.

grass_present es False por defecto (la cuchilla no enciende). Solo se pone en true con el argumento explicito del
launch `vision_stub_grass_present:=true` (pruebas de banco), y mientras este activo se avisa en el log cada 10 s.
"""
import time

from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from std_msgs.msg import Bool

from lawnbot_nav.node_utils import spin_with_cleanup


class VisionStub(Node):
    def __init__(self):
        super().__init__('vision_stub')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('grass_present', False)
        gp = self.get_parameter
        self.frame_id = gp('frame_id').value
        self.grass = gp('grass_present').value
        self.obstacles_pub = self.create_publisher(PoseArray, 'vision/obstacles', 10)
        self.grass_pub = self.create_publisher(Bool, 'vision/grass_present', 10)
        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.last_warn = 0.0
        self.warn_period = 10.0
        if self.grass:
            self.warn_grass()
        else:
            self.get_logger().info('vision_stub: obstaculos vacios, grass_present=false (la cuchilla no enciende)')

    def warn_grass(self):
        self.get_logger().warn(
            'vision_stub publica grass_present=TRUE sin camara: NO hay deteccion real de pasto. '
            'Solo para pruebas de banco con los motores desconectados.')
        self.last_warn = time.monotonic()

    def tick(self):
        if self.grass and time.monotonic() - self.last_warn >= self.warn_period:
            self.warn_grass()
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        self.obstacles_pub.publish(msg)
        self.grass_pub.publish(Bool(data=bool(self.grass)))

    def shutdown_cleanup(self):
        self.grass_pub.publish(Bool(data=False))          # sin camara no hay pasto


def main(args=None):
    spin_with_cleanup(VisionStub, args)


if __name__ == '__main__':
    main()
