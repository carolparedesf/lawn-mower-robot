#!/usr/bin/env python3
"""Vigilante del IMU: si /imu/data tiene mas de `imu_timeout` s sin mensajes, avisa `imu:imu_lost` por /nav/alert_in.

Con el EKF sin giroscopio el rumbo queda solo con la prediccion del filtro (ver docs/setup_navegacion.md): el safety_gate
detiene los modos automaticos mientras la alerta este vigente; el modo manual sigue libre. Mientras dura renueva la alerta
cada segundo (el gate la vence a los `alert_ttl` s) y al volver el IMU publica `imu:none`. imu_timeout <= 0 lo apaga.
"""
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.safety_logic import ALERT_IMU_LOST, ALERT_NONE, imu_is_lost


class ImuWatchdog(Node):
    def __init__(self):
        super().__init__('imu_watchdog')
        d = self.declare_parameter
        d('imu_topic', 'imu/data')
        d('imu_timeout', 0.5)               # s sin mensajes para dar el IMU por perdido (<= 0: apagado)
        d('rate_hz', 10.0)
        d('renew_period_s', 1.0)            # mientras dure la alerta se vuelve a publicar cada tanto
        gp = self.get_parameter
        self.timeout = gp('imu_timeout').value
        self.renew = gp('renew_period_s').value
        self.started = time.monotonic()
        self.last_msg = None
        self.alert_sent = None
        self.last_pub = 0.0
        self.pub = self.create_publisher(String, 'nav/alert_in', 10)
        self.create_subscription(Imu, gp('imu_topic').value, self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / gp('rate_hz').value, self.tick)
        self.get_logger().info(
            f'imu_watchdog listo: {gp("imu_topic").value}, alerta {ALERT_IMU_LOST} tras {self.timeout} s sin mensajes'
            if self.timeout > 0.0 else 'imu_watchdog apagado (imu_timeout <= 0)')

    def on_imu(self, msg):
        self.last_msg = time.monotonic()

    def tick(self):
        now = time.monotonic()
        alert = ALERT_IMU_LOST if imu_is_lost(now, self.last_msg, self.started, self.timeout) else ALERT_NONE
        if alert != self.alert_sent:
            if alert == ALERT_IMU_LOST:
                self.get_logger().warn('sin /imu/data: modos automaticos detenidos hasta que vuelva')
            elif self.alert_sent is not None:
                self.get_logger().info('/imu/data volvio')
        elif alert == ALERT_NONE or now - self.last_pub < self.renew:
            return
        self.alert_sent = alert
        self.last_pub = now
        self.pub.publish(String(data=f'imu:{alert}'))


def main(args=None):
    spin_with_cleanup(ImuWatchdog, args)


if __name__ == '__main__':
    main()
