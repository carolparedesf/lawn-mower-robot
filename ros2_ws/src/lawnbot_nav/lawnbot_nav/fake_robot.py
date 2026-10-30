#!/usr/bin/env python3
"""Robot simulado para probar sin hardware: lee una orden de velocidad y publica la TF.

NO ejecutar junto con ekf_local ni con la odometria real: habria dos fuentes del TF
frame_id -> child_frame (odom -> base_footprint).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from lawnbot_nav.sim_robot import SimRobot


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')

        d = self.declare_parameter
        d('frame_id', 'odom')
        d('child_frame', 'base_footprint')
        d('cmd_topic', 'cmd_vel_waypoint')   # 'cmd_vel' para pasar por twist_mux + safety_gate
        d('rate_hz', 50.0)
        d('cmd_timeout', 0.5)                # como el watchdog del firmware
        d('x', 0.0)
        d('y', 0.0)
        d('yaw', 0.0)
        d('base_width', 0.355)
        d('wheel_deadzone', 0.0)             # [m/s] por rueda; para probar min_linear y min_turn
        d('tau', 0.1)                        # lazo de velocidad [s]

        gp = self.get_parameter
        self.robot = SimRobot(gp('x').value, gp('y').value, gp('yaw').value,
                              base_width=gp('base_width').value,
                              wheel_deadzone=gp('wheel_deadzone').value,
                              tau=gp('tau').value)
        self.frame_id = gp('frame_id').value
        self.child_frame = gp('child_frame').value
        self.cmd_timeout = gp('cmd_timeout').value
        self.dt = 1.0 / gp('rate_hz').value
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd = None

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, gp('cmd_topic').value, self.on_cmd, 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            f"fake_robot: {self.frame_id} -> {self.child_frame}, orden en '{gp('cmd_topic').value}'")

    def on_cmd(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.last_cmd = time.monotonic()

    def tick(self):
        stale = self.last_cmd is None or time.monotonic() - self.last_cmd > self.cmd_timeout
        v, w = (0.0, 0.0) if stale else (self.v_cmd, self.w_cmd)
        self.robot.step(v, w, self.dt)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = self.robot.x
        tf.transform.translation.y = self.robot.y
        tf.transform.rotation.z = math.sin(self.robot.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.robot.yaw / 2.0)
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
