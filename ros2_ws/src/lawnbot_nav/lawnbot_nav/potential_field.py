#!/usr/bin/env python3
"""Evasion por campo de potencial en /cmd_vel_potential_field. Opera solo en auto_parallel y auto_random Y solo si
esta activado (arranca APAGADO): por el servicio /potential_field/set_enabled (std_srvs/SetBool) o por el topic
/potential_field/enable (Bool).

Toma la orden del algoritmo activo (/cmd_vel_waypoint, /cmd_vel_parallel o /cmd_vel_random), la corrige con la
repulsion de los sonares (y de /vision/obstacles solo si use_vision) y la publica con prioridad 60 en twist_mux,
mas que la del algoritmo y menos que la manual. Si este nodo falla, el mux vuelve solo al algoritmo (timeout 0,5 s).
La salida pasa siempre por twist_mux y safety_gate; este nodo nunca publica en /cmd_vel.
"""
import math
import time
from dataclasses import fields

from geometry_msgs.msg import PoseArray, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.potential_field_logic import PFCore, PFParams, sonar_point
from lawnbot_nav.sensor_cache import SONARS, SensorCache
from lawnbot_nav.tf_utils import lookup_pose

MODES = {'auto_parallel': 'nominal_topics_parallel', 'auto_random': 'nominal_topics_random'}
MODE_STALE_S = 3.0
STATUS_PERIOD_S = 1.0


class PotentialField(Node):
    def __init__(self):
        super().__init__('potential_field')
        d = self.declare_parameter
        d('rate_hz', 20.0)
        d('output_topic', 'cmd_vel_potential_field')
        d('enabled', False)                    # estado inicial; los reinicios siempre arrancan apagados salvo que se pida
        d('use_vision', False)                 # /vision/obstacles solo si la camara esta calibrada
        d('vision_topic', 'vision/obstacles')
        d('vision_timeout', 0.5)
        d('sensor_timeout', 1.0)
        d('nominal_timeout', 0.5)              # la orden del algoritmo mas vieja que esto no cuenta
        d('nominal_topics_parallel', ['cmd_vel_waypoint', 'cmd_vel_parallel'])
        d('nominal_topics_random', ['cmd_vel_random'])
        d('base_frame', 'base_footprint')
        d('progress_frame', 'odom')            # frame continuo (EKF local) para medir el progreso
        d('pose_timeout', 0.5)
        d('sonar_max_range', 1.5)              # lecturas >= (max - 0.05) = sin eco: no son obstaculo
        d('sonar_min_range', 0.02)
        # Montaje de cada sonar: x, y [m] y yaw [grados]. El URDF los pone todos al frente; los 45 grados laterales
        # son PROVISORIOS (igual que side_sonar_angle_deg de perimeter y fake_robot).
        d('sonar_left', [0.194, 0.136, 45.0])
        d('sonar_center', [0.198, 0.0, 0.0])
        d('sonar_right', [0.193, -0.1435, -45.0])
        defaults = PFParams()
        names = [f.name for f in fields(PFParams)]
        for n in names:
            d(n, getattr(defaults, n))

        gp = self.get_parameter
        self.core = PFCore(PFParams(**{n: gp(n).value for n in names}))
        self.enabled = bool(gp('enabled').value)
        self.use_vision = bool(gp('use_vision').value)
        self.vision_timeout, self.sensor_timeout = gp('vision_timeout').value, gp('sensor_timeout').value
        self.nominal_timeout = gp('nominal_timeout').value
        self.base_frame, self.progress_frame = gp('base_frame').value, gp('progress_frame').value
        self.pose_timeout = gp('pose_timeout').value
        self.max_range, self.min_range = gp('sonar_max_range').value, gp('sonar_min_range').value
        self.mounts = {s: (gp(f'sonar_{s}').value[0], gp(f'sonar_{s}').value[1], math.radians(gp(f'sonar_{s}').value[2]))
                       for s in SONARS}
        self.topics_by_mode = {m: list(gp(p).value) for m, p in MODES.items()}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cache = SensorCache()
        self.nominal = {}                      # topic -> (v, w, instante)
        self.vision = []                       # puntos (x, y) en base_frame
        self.vision_time = -math.inf
        self.active_mode, self.mode_time = None, 0.0
        self.was_active, self.zeros_pending = False, 0
        self.alert_sent, self.last_alert_pub, self.last_status_pub = None, 0.0, 0.0
        self.warned_no_pose = False

        self.pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.alert_pub = self.create_publisher(String, 'nav/alert_in', 10)
        self.status_pub = self.create_publisher(String, 'potential_field/status', 10)
        for s in SONARS:
            self.create_subscription(Range, f'sonar/{s}', lambda m, s=s: self.cache.set(f'sonar/{s}', m.range, time.monotonic()),
                                     qos_profile_sensor_data)
        for topic in {t for ts in self.topics_by_mode.values() for t in ts}:
            self.create_subscription(Twist, topic, lambda m, t=topic: self.nominal.__setitem__(
                t, (m.linear.x, m.angular.z, time.monotonic())), 10)
        if self.use_vision:
            self.create_subscription(PoseArray, gp('vision_topic').value, self.on_vision, 10)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, 10)
        self.create_subscription(Bool, 'potential_field/enable', lambda m: self.set_enabled(bool(m.data)), 10)
        self.create_service(SetBool, 'potential_field/set_enabled', self.on_set_enabled)
        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(
            f'potential_field listo: {"ACTIVADO" if self.enabled else "apagado"}, solo sonares'
            f'{" + vision" if self.use_vision else ""}, modos {sorted(MODES)}')

    # ── Entradas ─────────────────────────────────────────────

    def set_enabled(self, value):
        if value != self.enabled:
            self.get_logger().info('activado' if value else 'desactivado')
        self.enabled = value
        if not value:
            self.core.reset()

    def on_set_enabled(self, request, response):
        self.set_enabled(bool(request.data))
        response.success, response.message = True, 'activado' if self.enabled else 'desactivado'
        return response

    def on_mode(self, msg):
        self.active_mode, self.mode_time = msg.data, time.monotonic()

    def on_vision(self, msg):
        if msg.header.frame_id not in ('', self.base_frame):
            self.get_logger().warn(f'/vision/obstacles en frame {msg.header.frame_id!r}, se esperaba {self.base_frame!r}: se ignora',
                                   throttle_duration_sec=10.0)
            return
        self.vision = [(p.position.x, p.position.y) for p in msg.poses]
        self.vision_time = time.monotonic()

    # ── Ciclo ────────────────────────────────────────────────

    def mode_ok(self, now):
        return self.active_mode in MODES and now - self.mode_time <= MODE_STALE_S

    def sonar_obstacles(self, now):
        """Puntos detectados por los sonares, o None si falta alguna lectura vigente (falla segura)."""
        points = []
        for s in SONARS:
            r = self.cache.get(f'sonar/{s}', now, self.sensor_timeout)
            if r is None:
                return None
            if self.min_range <= r < self.max_range - 0.05:
                points.append(sonar_point(self.mounts[s], r))
        return points

    def nominal_command(self, now):
        best = None
        for topic in self.topics_by_mode[self.active_mode]:
            v, w, t = self.nominal.get(topic, (0.0, 0.0, -math.inf))
            if now - t <= self.nominal_timeout and (best is None or t > best[2]):
                best = (v, w, t)
        return None if best is None else (best[0], best[1])

    def tick(self):
        now = time.monotonic()
        active = self.enabled and self.mode_ok(now)
        if self.was_active and not active:
            self.zeros_pending = 3
            self.core.reset()
            self.set_alert(None, now, force=True)
        if active and not self.was_active:
            self.core.reset()
        self.was_active = active

        if active:
            obstacles = self.sonar_obstacles(now)
            if obstacles is not None:
                if self.use_vision and now - self.vision_time <= self.vision_timeout:
                    obstacles = obstacles + self.vision
                pose = lookup_pose(self, self.tf_buffer, self.progress_frame, self.base_frame, self.pose_timeout)
                if pose is None and not self.warned_no_pose:
                    self.warned_no_pose = True
                    self.get_logger().warn(f'sin TF {self.progress_frame}->{self.base_frame}: no se puede detectar el minimo local')
                out = self.core.update(now, self.nominal_command(now), obstacles,
                                       None if pose is None else (pose.x, pose.y))
                if out is not None:
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = float(out[0]), float(out[1])
                    self.pub.publish(cmd)
                self.set_alert(self.core.alert, now)
            # sin sonares vigentes no se publica: el mux cae al algoritmo y el gate lo frena por sensor viejo
        elif self.zeros_pending > 0:
            self.zeros_pending -= 1
            self.pub.publish(Twist())
        self.publish_status(now, active)

    def set_alert(self, alert, now, force=False):
        if alert == self.alert_sent and not (alert and now - self.last_alert_pub >= 1.0) and not force:
            return
        self.alert_sent, self.last_alert_pub = alert, now
        self.alert_pub.publish(String(data=f'potential_field:{alert or "none"}'))

    def publish_status(self, now, active):
        if now - self.last_status_pub < STATUS_PERIOD_S:
            return
        self.last_status_pub = now
        state = 'off' if not self.enabled else ('waiting_mode' if not active else
                                                ('yield' if self.core.alert else
                                                 ('avoiding' if self.core.influenced else 'on')))
        self.status_pub.publish(String(data=state))

    def shutdown_cleanup(self):
        for _ in range(5):
            try:
                self.pub.publish(Twist())
            except Exception:
                break
            time.sleep(0.02)


def main(args=None):
    spin_with_cleanup(PotentialField, args)


if __name__ == '__main__':
    main()
