#!/usr/bin/env python3
"""Seguidor de waypoints (pure pursuit). Publica en /cmd_vel_waypoint (entrada de twist_mux).

No arranca solo: espera std_msgs/Bool true en /follower/enable. La logica esta en pure_pursuit.py.
Se pausa (y exige `false` + `true` para reanudar) ante: cualquier mensaje en /cmd_vel_manual,
/estop true, o salir de allowed_modes. Con TF caida o pose vieja publica cero.
"""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from lawnbot_nav.pure_pursuit import (RUNNING, FollowerParams, Pose,
                                      WaypointFollowerCore)

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODE_STALE_S = 3.0   # /nav/active_mode se republica cada 1 s


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        d = self.declare_parameter
        d('frame_id', 'odom')
        d('base_frame', 'base_footprint')
        d('waypoints', [0.0, 0.0])           # lista plana [x0, y0, x1, y1, ...] en metros
        d('relative_to_start', True)
        d('loop', False)
        d('lookahead_distance', 0.5)
        d('max_linear', 0.2)
        d('min_linear', 0.0)
        d('max_angular', 0.6)
        d('min_turn', 0.0)
        d('goal_tolerance', 0.2)
        d('rotate_in_place_angle', 0.7)
        d('rotate_exit_ratio', 0.5)
        d('turn_gain', 1.5)
        d('slowdown_radius', 0.6)
        d('progress_timeout', 20.0)
        d('tf_abort_s', 3.0)
        d('pose_timeout', 0.5)
        d('rate_hz', 20.0)
        d('allowed_modes', ['manual'])
        d('require_active_mode', True)
        d('pause_on_manual', True)
        d('output_topic', 'cmd_vel_waypoint')

        gp = self.get_parameter
        flat = [float(x) for x in gp('waypoints').value]
        if len(flat) % 2 != 0:
            raise ValueError('waypoints debe tener cantidad par de valores [x0, y0, x1, y1, ...]')
        waypoints = list(zip(flat[0::2], flat[1::2]))

        self.core = WaypointFollowerCore(FollowerParams(
            waypoints=waypoints,
            relative_to_start=gp('relative_to_start').value,
            loop=gp('loop').value,
            lookahead_distance=gp('lookahead_distance').value,
            max_linear=gp('max_linear').value,
            min_linear=gp('min_linear').value,
            max_angular=gp('max_angular').value,
            min_turn=gp('min_turn').value,
            goal_tolerance=gp('goal_tolerance').value,
            rotate_in_place_angle=gp('rotate_in_place_angle').value,
            rotate_exit_ratio=gp('rotate_exit_ratio').value,
            turn_gain=gp('turn_gain').value,
            slowdown_radius=gp('slowdown_radius').value,
            progress_timeout=gp('progress_timeout').value,
            tf_abort_s=gp('tf_abort_s').value))

        self.frame_id = gp('frame_id').value
        self.base_frame = gp('base_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.allowed_modes = list(gp('allowed_modes').value)
        self.require_mode = gp('require_active_mode').value
        self.pause_on_manual = gp('pause_on_manual').value

        self.active_mode = None
        self.active_mode_time = 0.0
        self.estop = False
        self.last_state = None
        self.zeros_pending = 0   # ceros a publicar tras dejar RUNNING

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.state_pub = self.create_publisher(String, 'follower/state', LATCHED)
        self.create_subscription(Bool, 'follower/enable', self.on_enable, 10)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_subscription(Twist, 'cmd_vel_manual', self.on_manual, 10)
        self.create_subscription(Bool, 'estop', self.on_estop, 10)

        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.create_timer(1.0, self.log_status)
        self.publish_state()
        self.get_logger().info(
            f'waypoint_follower listo: {len(waypoints)} waypoints, frame {self.frame_id}, '
            f'relative_to_start={self.core.p.relative_to_start}, esperando /follower/enable')

    # ── Entradas ─────────────────────────────────────────────

    def on_enable(self, msg):
        if not msg.data:
            self.core.disable()
            self.send_zero()
            self.get_logger().info('enable=false: PAUSED')
            return
        if self.estop:
            self.get_logger().warn('enable rechazado: /estop activo')
            return
        if not self.mode_ok():
            self.get_logger().warn(
                f"enable rechazado: modo activo '{self.active_mode}' no esta en {self.allowed_modes}")
            return
        pose = self.get_pose()
        if pose is None:
            self.get_logger().warn(f'enable rechazado: sin pose {self.frame_id} -> {self.base_frame}')
            return
        self.get_logger().info(f'enable: {self.core.enable(pose, time.monotonic())}')

    def on_mode(self, msg):
        self.active_mode = msg.data
        self.active_mode_time = time.monotonic()

    def on_manual(self, msg):
        # Cualquier mensaje manual (incluido un cero) pausa: el operador toma el control
        if self.pause_on_manual and self.core.state == RUNNING:
            self.core.pause('manual')
            self.send_zero()
            self.get_logger().warn('comando manual recibido: PAUSED (publicar enable false y luego true)')

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop and self.core.state == RUNNING:
            self.core.pause('estop')
            self.send_zero()
            self.get_logger().warn('/estop activo: PAUSED (publicar enable false y luego true)')

    # ── Utilidades ───────────────────────────────────────────

    def mode_ok(self):
        if not self.require_mode:
            return True
        fresh = time.monotonic() - self.active_mode_time <= MODE_STALE_S
        return fresh and self.active_mode in self.allowed_modes

    def get_pose(self):
        """Pose de base_frame en frame_id, o None si falta la TF o es mas vieja que pose_timeout."""
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time())
        except TransformException:
            return None
        stamp = tf.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:   # stamp 0 = transformada estatica
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9
            if age > self.pose_timeout:
                return None
        t = tf.transform
        return Pose(t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))

    def send_zero(self):
        self.pub.publish(Twist())

    # ── Ciclo ────────────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        if self.core.state == RUNNING and not self.mode_ok():
            self.core.pause('modo')
            self.get_logger().warn(
                f"modo activo '{self.active_mode}' fuera de {self.allowed_modes}: PAUSED "
                '(publicar enable false y luego true)')

        v = w = 0.0
        if self.core.state == RUNNING:
            pose = self.get_pose()
            if pose is None:
                v, w = self.core.pose_unavailable(now)
            else:
                v, w = self.core.update(pose, now)

        if self.core.state == RUNNING:
            self.zeros_pending = 3
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.pub.publish(cmd)
        elif self.zeros_pending > 0:
            # Al dejar RUNNING (pausa, fin o error) se publica cero unas veces; luego silencio
            self.zeros_pending -= 1
            self.send_zero()

        if self.core.state != self.last_state:
            if self.core.state not in (RUNNING,) and self.core.reason:
                self.get_logger().info(f'{self.core.state}: {self.core.reason}')
            self.last_state = self.core.state
            self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.core.state))

    def log_status(self):
        c = self.core
        if c.state != RUNNING:
            self.state_pub.publish(String(data=c.state))
            return
        i = c.info
        if not i:
            return
        self.get_logger().info(
            f"waypoint {i['k'] + 1}/{i['n']} | dist {i['dist']:.2f} m | "
            f"error rumbo {math.degrees(i['alpha']):6.1f} deg | "
            f"v {i['v']:.2f} m/s  w {i['w']:.2f} rad/s | {i['phase']}")
        self.state_pub.publish(String(data=c.state))

    def shutdown_cleanup(self):
        """Publica cero varias veces antes de apagar."""
        for _ in range(5):
            try:
                self.send_zero()
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    # Sin el manejador de senales de rclpy para poder publicar cero antes de cerrar
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def _stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
