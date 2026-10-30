#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.0925   # m
BASE_WIDTH   = 0.355    # m

class JointStateFromOdom(Node):
    def __init__(self):
        super().__init__('joint_state_from_odom')

        self.declare_parameter('odom_topic', '/odom/unfiltered')
        self.declare_parameter('left_joint',  'base_BLwheel_joint')
        self.declare_parameter('right_joint', 'base_BRwheel_joint')

        odom_topic  = self.get_parameter('odom_topic').value
        self.left_joint  = self.get_parameter('left_joint').value
        self.right_joint = self.get_parameter('right_joint').value

        self.pos_left  = 0.0
        self.pos_right = 0.0
        self.last_time = None

        self.sub = self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Cinemática diferencial inversa: velocidad lineal de cada rueda
        v_right = v + w * BASE_WIDTH / 2.0
        v_left  = v - w * BASE_WIDTH / 2.0

        # Velocidad angular de cada rueda -> integración a posición
        w_right = v_right / WHEEL_RADIUS
        w_left  = v_left  / WHEEL_RADIUS

        self.pos_right += w_right * dt
        self.pos_left  += w_left  * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.pos_left, self.pos_right]
        js.velocity = [w_left, w_right]
        self.pub.publish(js)

def main():
    rclpy.init()
    node = JointStateFromOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()