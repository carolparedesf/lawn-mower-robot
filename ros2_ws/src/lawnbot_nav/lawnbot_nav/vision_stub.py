#!/usr/bin/env python3
"""Reemplazo de la camara mientras no este calibrada: publica /vision/obstacles vacio y /vision/grass_present fijo.

grass_present arranca en False (la cuchilla no enciende). Ponerlo en true solo para pruebas de banco.
"""
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
        self.get_logger().info(f'vision_stub: obstaculos vacios, grass_present={self.grass}')

    def tick(self):
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
