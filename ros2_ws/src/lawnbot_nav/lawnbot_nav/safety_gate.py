#!/usr/bin/env python3
"""Ultima barrera antes del firmware: filtra /cmd_vel_mux y publica /cmd_vel.

Solo limita o anula velocidades, nunca las amplifica. Publica a ritmo fijo; si la entrada
esta vieja publica cero. Con la alerta imu_lost (/nav/alert_in, la publica imu_watchdog) y un modo automatico vigente publica cero.
Fuera del cerco (/geofence/inside false, con la histeresis de geofence_node) y un modo automatico vigente publica cero y la
alerta outside_fence; en manual deja mover (para volver a meter el robot; la cuchilla la apaga blade_controller) y
avisa outside_fence igual si el cerco esta OK. Las reglas estan en safety_logic.py y los parametros en el YAML.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from lawnbot_nav.safety_logic import (ALERT_OUTSIDE_FENCE, SafetyParams, SafetyState, SonarLimiter,
                                      apply_safety, fence_alert, fence_blocks_motion, ignored_sonars,
                                      imu_blocks_motion, merge_alerts, parse_alert_in)

LATCHED = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)

SONARS = ('left', 'center', 'right')


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')

        self.declare_parameter('input_topic', 'cmd_vel_mux')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('sonar_topic_prefix', 'sonar')   # 'sonar_filtered' con el filtro de sonares (nav.launch.py)
        self.declare_parameter('alert_ttl', 3.0)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('sonar_slow_distance', 0.30)
        self.declare_parameter('sonar_stop_distance', 0.15)
        self.declare_parameter('sonar_stop_hysteresis', 0.05)
        self.declare_parameter('sonar_release_accel', 0.5)
        for s in SONARS:
            self.declare_parameter(f'sonar_{s}_enabled', True)
            self.declare_parameter(f'sonar_{s}_topic', '')           # vacio = <sonar_topic_prefix>/<s>
            # Umbrales propios de cada sonar; negativo = usar los globales
            self.declare_parameter(f'sonar_{s}_slow_distance', -1.0)
            self.declare_parameter(f'sonar_{s}_stop_distance', -1.0)
        # En auto_perimeter los sonares laterales no cuentan (el central y los bumpers siguen protegiendo)
        self.declare_parameter('ignore_side_sonars_in_perimeter', True)
        self.declare_parameter('mode_timeout', 3.0)
        # Freno por cerco en modos automaticos; /geofence/inside sin llegar en este tiempo [s] cuenta como afuera
        self.declare_parameter('geofence_brake', True)
        self.declare_parameter('geofence_timeout', 1.0)
        self.declare_parameter('bumper_front_topics',
                               ['bumper/B1_front_left', 'bumper/B2_front_right'])
        self.declare_parameter('bumper_back_topics', ['bumper/B5_back'])
        self.declare_parameter('bumper_monitor_topics',
                               ['bumper/B3_left', 'bumper/B4_right'])

        gp = self.get_parameter
        self.params = SafetyParams(
            sonar_slow_distance=gp('sonar_slow_distance').value,
            sonar_stop_distance=gp('sonar_stop_distance').value,
            max_linear=gp('max_linear').value,
            max_angular=gp('max_angular').value,
            sonar_stop_hysteresis=gp('sonar_stop_hysteresis').value,
            sonar_release_accel=gp('sonar_release_accel').value)
        self.limiter = SonarLimiter()
        for s in SONARS:
            slow = gp(f'sonar_{s}_slow_distance').value
            stop = gp(f'sonar_{s}_stop_distance').value
            if slow >= 0.0 or stop >= 0.0:
                self.params.sonar_thresholds[s] = (
                    slow if slow >= 0.0 else self.params.sonar_slow_distance,
                    stop if stop >= 0.0 else self.params.sonar_stop_distance)
        self.ignore_sides = gp('ignore_side_sonars_in_perimeter').value
        self.mode_timeout = gp('mode_timeout').value
        self.active_mode = None
        self.mode_time = 0.0
        self.ignoring = ()
        self.input_timeout = gp('input_timeout').value
        self.sensor_timeout = gp('sensor_timeout').value
        self.alert_ttl = gp('alert_ttl').value
        self.external_alerts = {}     # fuente -> (alerta, instante de recepcion)
        self.geofence_brake = gp('geofence_brake').value
        self.geofence_timeout = gp('geofence_timeout').value
        self.inside = None            # ultimo /geofence/inside
        self.inside_time = 0.0
        self.fence_status = None      # ultimo /geofence/status (None = geofence_node nunca publico)
        self.fence_blocking = False

        self.v_in = 0.0
        self.w_in = 0.0
        self.last_in = None

        # Estado de sensores: nombre -> (valor, instante de recepcion)
        self.sonar = {}
        self.bumper = {}
        self.sonar_enabled = {s: gp(f'sonar_{s}_enabled').value for s in SONARS}
        self.front_topics = list(gp('bumper_front_topics').value)
        self.back_topics = list(gp('bumper_back_topics').value)
        self.monitor_topics = list(gp('bumper_monitor_topics').value)

        for s in SONARS:
            if self.sonar_enabled[s]:
                topic = gp(f'sonar_{s}_topic').value or f'{gp("sonar_topic_prefix").value}/{s}'
                self.create_subscription(
                    Range, topic,
                    lambda msg, s=s: self.on_sonar(s, msg), qos_profile_sensor_data)
        for topic in self.front_topics + self.back_topics + self.monitor_topics:
            self.create_subscription(
                Bool, topic, lambda msg, t=topic: self.on_bumper(t, msg),
                qos_profile_sensor_data)

        self.create_subscription(Twist, gp('input_topic').value, self.on_input, 10)
        self.create_subscription(String, 'nav/alert_in', self.on_alert_in, 10)
        self.create_subscription(String, 'nav/active_mode', self.on_mode, LATCHED)
        self.create_subscription(Bool, 'geofence/inside', self.on_inside, 10)
        self.create_subscription(String, 'geofence/status', self.on_fence_status, LATCHED)
        self.pub = self.create_publisher(Twist, gp('output_topic').value, 10)
        self.alert_pub = self.create_publisher(String, 'nav/safety_alert', LATCHED)

        self.alert = None
        self.last_alert_pub = 0.0
        self.create_timer(1.0 / gp('publish_rate_hz').value, self.tick)

        enabled = [s for s in SONARS if self.sonar_enabled[s]]
        self.get_logger().info(
            f'safety_gate listo: sonares {enabled}, bumpers frontales {self.front_topics}, '
            f'traseros {self.back_topics}, solo monitoreo {self.monitor_topics}')

    # ── Entradas ─────────────────────────────────────────────

    def on_input(self, msg):
        self.v_in = msg.linear.x
        self.w_in = msg.angular.z
        self.last_in = time.monotonic()

    def on_mode(self, msg):
        self.active_mode = msg.data
        self.mode_time = time.monotonic()

    def on_inside(self, msg):
        self.inside = bool(msg.data)
        self.inside_time = time.monotonic()

    def on_fence_status(self, msg):
        self.fence_status = msg.data

    def on_alert_in(self, msg):
        source, alert = parse_alert_in(msg.data)
        self.external_alerts[source] = (alert, time.monotonic())

    def on_sonar(self, name, msg):
        self.sonar[name] = (float(msg.range), time.monotonic())

    def on_bumper(self, topic, msg):
        prev = self.bumper.get(topic, (None, 0.0))[0]
        self.bumper[topic] = (bool(msg.data), time.monotonic())
        if topic in self.monitor_topics and prev != bool(msg.data):
            self.get_logger().info(f'{topic}: {"presionado" if msg.data else "libre"} (solo monitoreo)')

    # ── Estado de seguridad ──────────────────────────────────

    def fresh(self, entry, now):
        return entry is not None and now - entry[1] <= self.sensor_timeout

    def build_state(self, now, active_alerts=()):
        ranges = {}
        stale = False
        ignoring = ignored_sonars(self.ignore_sides, self.active_mode,
                                  now - self.mode_time <= self.mode_timeout)
        if ignoring != self.ignoring:
            self.ignoring = ignoring
            self.get_logger().info(
                f'sonares ignorados: {list(ignoring)}' if ignoring else 'se vuelven a considerar todos los sonares')
        for s in SONARS:
            if not self.sonar_enabled[s] or s in ignoring:
                continue
            entry = self.sonar.get(s)
            if self.fresh(entry, now):
                ranges[s] = entry[0]
            else:
                stale = True

        front = False
        for topic in self.front_topics:
            entry = self.bumper.get(topic)
            if self.fresh(entry, now):
                front = front or entry[0]
            else:
                stale = True

        # Un bumper trasero viejo no bloquea (solo se bloquea el avance por sensores viejos)
        back = any(self.fresh(self.bumper.get(t), now) and self.bumper[t][0]
                   for t in self.back_topics)

        mode_fresh = now - self.mode_time <= self.mode_timeout
        imu_lost_auto = imu_blocks_motion(active_alerts, self.active_mode, mode_fresh)
        outside = self.geofence_brake and fence_blocks_motion(
            self.inside, now - self.inside_time, self.fence_status, self.active_mode, mode_fresh, self.geofence_timeout)
        if outside != self.fence_blocking:
            self.fence_blocking = outside
            if outside:
                self.get_logger().warn(f'fuera del cerco en {self.active_mode}: se frena (outside_fence)')
            else:
                self.get_logger().info('freno por cerco liberado')
        return SafetyState(sonar_ranges=ranges, front_pressed=front, back_pressed=back, stale=stale,
                           imu_lost_auto=imu_lost_auto, outside_fence_auto=outside)

    # ── Salida ───────────────────────────────────────────────

    def tick(self):
        now = time.monotonic()
        if self.last_in is None or now - self.last_in > self.input_timeout:
            v_cmd, w_cmd = 0.0, 0.0
        else:
            v_cmd, w_cmd = self.v_in, self.w_in

        external = {src: (alert, now - t) for src, (alert, t) in self.external_alerts.items()}
        active = {alert for alert, age in external.values() if age <= self.alert_ttl}
        state = self.build_state(now, active)
        v, w, local_alert = apply_safety(v_cmd, w_cmd, state, self.params, limiter=self.limiter, now=now)
        if fence_alert(state.outside_fence_auto, self.inside, now - self.inside_time, self.fence_status,
                       self.geofence_timeout):
            external['safety_gate:geofence'] = (ALERT_OUTSIDE_FENCE, 0.0)
        alert = merge_alerts(local_alert, external, self.alert_ttl)

        out = Twist()
        out.linear.x = float(v)
        out.angular.z = float(w)
        self.pub.publish(out)

        if alert != self.alert or now - self.last_alert_pub >= 1.0:
            if alert != self.alert:
                self.get_logger().info(f'alerta: {alert}')
            self.alert = alert
            self.last_alert_pub = now
            self.alert_pub.publish(String(data=alert))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
