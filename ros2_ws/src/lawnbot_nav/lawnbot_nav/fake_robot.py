#!/usr/bin/env python3
"""Robot simulado para probar sin hardware: lee una orden de velocidad y publica la TF.

Opcionalmente (walls / obstacles / publish_sensors) vive en un mundo con paredes y obstaculos
circulares y publica los sonares y bumpers como el firmware, con colisiones.

NO ejecutar junto con ekf_local ni con la odometria real: habria dos fuentes del TF
frame_id -> child_frame (odom -> base_footprint).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_world import (BUMPER_MOUNTS, SONAR_MAX_RANGE, SONAR_MIN_RANGE, SONAR_FOV,
                                   SimWorld)

SONAR_FRAMES = {'left': 'SonarL_Link', 'center': 'SonarC_Link', 'right': 'SonarR_Link'}


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
        # Mundo simulado (opcional). Listas planas: walls = [x1, y1, x2, y2, ...] (un segmento cada 4),
        # obstacles = [x, y, radio, ...]. Valores en cero se ignoran.
        d('walls', [0.0, 0.0, 0.0, 0.0])
        d('obstacles', [0.0, 0.0, 0.0])
        # Esquinas redondeadas del cuerpo y como se reparten los bumpers (ver sim_world.bumper_zones). Radio 0 = rectangulo
        d('corner_radius', 0.0)              # [m]
        d('corner_split_deg', 45.0)          # [grados] desde la cara frontal/trasera: hasta aqui el bumper frontal/trasero, luego el lateral
        d('corner_dead_deg', 0.0)            # [grados] zona sin bumper centrada en el split
        d('publish_sensors', False)
        d('sensor_rate_hz', 5.0)             # como el firmware
        d('side_sonar_yaw_deg', 0.0)         # angulo de los sonares laterales (el URDF los pone en 0)

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

        w = [float(v) for v in gp('walls').value]
        o = [float(v) for v in gp('obstacles').value]
        walls = [((w[i], w[i + 1]), (w[i + 2], w[i + 3])) for i in range(0, len(w) - 3, 4)
                 if (w[i], w[i + 1]) != (w[i + 2], w[i + 3])]
        circles = [(o[i], o[i + 1], o[i + 2]) for i in range(0, len(o) - 2, 3)]
        self.world = (SimWorld(walls, circles, corner_radius=gp('corner_radius').value,
                               corner_split_deg=gp('corner_split_deg').value,
                               corner_dead_deg=gp('corner_dead_deg').value) if (walls or circles) else None)
        yaw = gp('side_sonar_yaw_deg').value
        self.sonar_yaw = {'left': yaw, 'center': 0.0, 'right': -yaw}

        self.broadcaster = TransformBroadcaster(self)
        if gp('publish_sensors').value:
            self.sonar_pubs = {s: self.create_publisher(Range, f'sonar/{s}', 10) for s in SONAR_FRAMES}
            self.bumper_pubs = {n: self.create_publisher(Bool, n, 10) for n in BUMPER_MOUNTS}
            self.create_timer(1.0 / gp('sensor_rate_hz').value, self.publish_sensors)
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
        if self.world is not None:
            self.world.step_robot(self.robot, v, w, self.dt)
        else:
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


    def publish_sensors(self):
        if self.world is None:
            return
        pose = (self.robot.x, self.robot.y, self.robot.yaw)
        stamp = self.get_clock().now().to_msg()
        for name, value in self.world.sonars(pose, self.sonar_yaw).items():
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = SONAR_FRAMES[name]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = SONAR_FOV
            msg.min_range = SONAR_MIN_RANGE
            msg.max_range = SONAR_MAX_RANGE
            msg.range = float(value)
            self.sonar_pubs[name].publish(msg)
        for name, pressed in self.world.bumpers(pose).items():
            self.bumper_pubs[name].publish(Bool(data=bool(pressed)))


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
