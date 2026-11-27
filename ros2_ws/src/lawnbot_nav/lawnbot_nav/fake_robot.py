#!/usr/bin/env python3
"""Robot simulado para probar sin hardware: lee una orden de velocidad y publica la TF.

Opcionalmente (walls / obstacles / publish_sensors) vive en un mundo con paredes y obstaculos
circulares y publica los sonares y bumpers como el firmware, con colisiones.

Con `publish_markers` (solo simulacion; apagado por defecto, lo enciende config/sim_room.yaml) publica para RViz
/sim/markers (paredes, obstaculos, huella con esquinas redondeadas y cada bumper, rojo al presionarse) y
/sim/path (rastro). Ver config/sim.rviz.

NO ejecutar junto con ekf_local ni con la odometria real: habria dos fuentes del TF
frame_id -> child_frame (odom -> base_footprint).
"""
import math
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from lawnbot_nav.sim_robot import SimRobot
from lawnbot_nav.sim_view import Trail, bumper_polylines, footprint
from lawnbot_nav.robot_geometry import (BODY_FRONT_DEPTH, BODY_FRONT_WIDTH, BODY_LENGTH, BODY_REAR_WIDTH,
                                        CORNER_SONAR_ANGLE_DEG, sonar_yaws)
from lawnbot_nav.sim_world import (Footprint, BUMPER_MOUNTS, SONAR_MAX_RANGE, SONAR_MIN_RANGE, SONAR_FOV,
                                   SimWorld)

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

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
        # Pose simulada tambien como nav_msgs/Odometry (lo que en el robot publica ekf_global), solo con frame_id
        # 'map': coverage_map y el bridge leen la pose de ahi y no de la TF. '' = no publica
        d('odom_topic', 'odometry/global')
        d('odom_rate_hz', 10.0)              # como ekf_global
        # Visualizacion en RViz (solo simulacion): /sim/markers (MarkerArray) y /sim/path (Path), en `frame_id`
        d('publish_markers', False)
        d('markers_rate_hz', 1.0)
        d('path_max_poses', 2000)
        d('path_min_step', 0.02)             # [m] distancia minima entre poses del rastro
        d('path_min_yaw', 0.087)             # [rad] o giro minimo
        d('sensor_rate_hz', 5.0)             # como el firmware
        d('imu_rate_hz', 20.0)               # /imu/data simulado (con publish_sensors) para que imu_watchdog no corte los modos automaticos; 0 = no publica
        # Angulo de los sonares de las esquinas: parametro compartido (config/robot_geometry.yaml), igual que el URDF,
        # potential_field, coverage_map y perimeter
        d('corner_sonar_angle_deg', CORNER_SONAR_ANGLE_DEG)
        # Huella: 'rectangle' (cuerpo rectangular de antes) o 'chassis' (huella del plano CAD, ver robot_geometry.py)
        d('body_shape', 'rectangle')
        d('body_length', BODY_LENGTH)
        d('body_front_depth', BODY_FRONT_DEPTH)
        d('body_front_width', BODY_FRONT_WIDTH)
        d('body_rear_width', BODY_REAR_WIDTH)

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
        shape = gp('body_shape').value
        if shape not in ('rectangle', 'chassis'):
            raise ValueError("body_shape debe ser 'rectangle' o 'chassis'")
        footprint = None
        if shape == 'chassis':
            footprint = Footprint(length=gp('body_length').value, front_depth=gp('body_front_depth').value,
                                  front_width=gp('body_front_width').value, rear_width=gp('body_rear_width').value,
                                  corner_radius=gp('corner_radius').value, split_deg=gp('corner_split_deg').value,
                                  dead_deg=gp('corner_dead_deg').value)
        self.world = (SimWorld(walls, circles, corner_radius=gp('corner_radius').value,
                               corner_split_deg=gp('corner_split_deg').value,
                               corner_dead_deg=gp('corner_dead_deg').value, footprint=footprint)
                      if (walls or circles) else None)
        self.sonar_yaw = sonar_yaws(gp('corner_sonar_angle_deg').value)

        self.broadcaster = TransformBroadcaster(self)
        self.view = None
        if gp('publish_markers').value:
            self.setup_view(gp)
        if gp('publish_sensors').value:
            self.sonar_pubs = {s: self.create_publisher(Range, f'sonar/{s}', 10) for s in SONAR_FRAMES}
            self.bumper_pubs = {n: self.create_publisher(Bool, n, 10) for n in BUMPER_MOUNTS}
            self.create_timer(1.0 / gp('sensor_rate_hz').value, self.publish_sensors)
            if gp('imu_rate_hz').value > 0.0:
                self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
                self.create_timer(1.0 / gp('imu_rate_hz').value, self.publish_imu)
        self.create_subscription(Twist, gp('cmd_topic').value, self.on_cmd, 10)
        self.create_timer(self.dt, self.tick)
        self.odom_pub = None
        if gp('odom_topic').value and self.frame_id == 'map' and gp('odom_rate_hz').value > 0.0:
            self.odom_pub = self.create_publisher(Odometry, gp('odom_topic').value, 10)
            self.create_timer(1.0 / gp('odom_rate_hz').value, self.publish_odom)
        self.get_logger().info(
            f"fake_robot: {self.frame_id} -> {self.child_frame}, orden en '{gp('cmd_topic').value}'")

    # ── Vista en RViz ────────────────────────────────────────

    def setup_view(self, gp):
        self.markers_pub = self.create_publisher(MarkerArray, 'sim/markers', LATCHED)
        self.path_pub = self.create_publisher(Path, 'sim/path', LATCHED)
        self.trail = Trail(gp('path_max_poses').value, gp('path_min_step').value, gp('path_min_yaw').value)
        radius = self.world.corner_radius if self.world is not None else 0.0
        fp = self.world.footprint if self.world is not None else None
        self.view = {'footprint': (fp.outline + fp.outline[:1]) if fp is not None else footprint(radius),
                     'bumpers': bumper_polylines(self.world.zones) if self.world is not None else {}}
        self.pressed = {}
        self.create_timer(1.0 / gp('markers_rate_hz').value, self.publish_view)
        self.get_logger().info('fake_robot: publica /sim/markers y /sim/path para RViz (solo simulacion)')

    def _marker(self, ns, mid, mtype, frame, rgb, width):
        m = Marker()
        m.header.frame_id = frame                       # stamp 0: RViz usa la ultima TF disponible
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        return m

    def build_markers(self, pressed):
        array = MarkerArray()
        if self.world is not None:
            walls = self._marker('walls', 0, Marker.LINE_LIST, self.frame_id, (0.85, 0.85, 0.85), 0.05)
            for a, b in self.world.walls:
                walls.points += [Point(x=float(a[0]), y=float(a[1]), z=0.0), Point(x=float(b[0]), y=float(b[1]), z=0.0)]
            array.markers.append(walls)
            for i, (cx, cy, r) in enumerate(self.world.circles):
                m = self._marker('obstacles', i, Marker.CYLINDER, self.frame_id, (0.85, 0.55, 0.2), 2.0 * r)
                m.scale.y, m.scale.z = 2.0 * r, 0.3
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(cx), float(cy), 0.15
                array.markers.append(m)
        body = self._marker('footprint', 0, Marker.LINE_STRIP, self.child_frame, (1.0, 1.0, 1.0), 0.01)
        body.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in self.view['footprint']]
        array.markers.append(body)
        for i, (name, pts) in enumerate(self.view['bumpers'].items()):
            rgb = (1.0, 0.1, 0.1) if pressed.get(name) else (0.25, 0.7, 0.3)
            m = self._marker('bumpers', i, Marker.LINE_STRIP, self.child_frame, rgb, 0.03)
            m.points = [Point(x=float(x), y=float(y), z=0.01) for x, y in pts]
            array.markers.append(m)
        return array

    def build_path(self):
        msg = Path()
        msg.header.frame_id = self.frame_id
        for x, y, yaw in self.trail.poses:
            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
            msg.poses.append(p)
        return msg

    def current_bumpers(self):
        if self.world is None:
            return {}
        return self.world.bumpers((self.robot.x, self.robot.y, self.robot.yaw))

    def publish_view(self):
        self.pressed = self.current_bumpers()
        self.markers_pub.publish(self.build_markers(self.pressed))
        self.path_pub.publish(self.build_path())

    def on_bumpers(self, pressed):
        """Se llama con cada lectura de bumpers: si cambia alguno se republica enseguida (el timer es de ~1 Hz)."""
        if self.view is not None and pressed != self.pressed:
            self.pressed = dict(pressed)
            self.markers_pub.publish(self.build_markers(self.pressed))

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
        if self.view is not None:
            self.trail.add(self.robot.x, self.robot.y, self.robot.yaw)


    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x = self.robot.x
        msg.pose.pose.position.y = self.robot.y
        msg.pose.pose.orientation.z = math.sin(self.robot.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.robot.yaw / 2.0)
        self.odom_pub.publish(msg)

    def publish_imu(self):
        """IMU ideal: yaw absoluto y velocidad angular z del robot simulado."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'BerryIMU_Link'
        msg.orientation.z = math.sin(self.robot.yaw / 2.0)
        msg.orientation.w = math.cos(self.robot.yaw / 2.0)
        msg.angular_velocity.z = float(getattr(self.robot, 'w', 0.0))
        self.imu_pub.publish(msg)

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
        bumpers = self.world.bumpers(pose)
        for name, pressed in bumpers.items():
            self.bumper_pubs[name].publish(Bool(data=bool(pressed)))
        self.on_bumpers(bumpers)


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
