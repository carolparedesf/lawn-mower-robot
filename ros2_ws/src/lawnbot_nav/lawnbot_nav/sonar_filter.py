#!/usr/bin/env python3
"""Filtra los sonares: sonar/<left|center|right> -> sonar_filtered/<left|center|right> (ver sonar_filter_logic.py).

Conserva el header y el stamp de la ultima lectura. Una lectura invalida no se publica: si el sonar deja de dar
datos validos, los consumidores ven el dato viejo y fallan seguro (safety_gate bloquea el avance).
"""
from dataclasses import fields

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from lawnbot_nav.node_utils import spin_with_cleanup
from lawnbot_nav.sonar_filter_logic import SonarFilter, SonarFilterParams

SONARS = ('left', 'center', 'right')


class SonarFilterNode(Node):
    def __init__(self):
        super().__init__('sonar_filter')
        defaults = SonarFilterParams()
        names = [f.name for f in fields(SonarFilterParams)]
        for n in names:
            self.declare_parameter(n, getattr(defaults, n))
        self.declare_parameter('input_prefix', 'sonar')
        self.declare_parameter('output_prefix', 'sonar_filtered')
        gp = self.get_parameter
        self.params = SonarFilterParams(**{n: gp(n).value for n in names})
        self.filters = {s: SonarFilter(self.params) for s in SONARS}
        self.out_pubs = {}
        for s in SONARS:
            self.out_pubs[s] = self.create_publisher(Range, f'{gp("output_prefix").value}/{s}', qos_profile_sensor_data)
            self.create_subscription(Range, f'{gp("input_prefix").value}/{s}',
                                     lambda msg, s=s: self.on_range(s, msg), qos_profile_sensor_data)
        self.get_logger().info(
            f'sonar_filter listo: mediana de {self.params.window}, salto maximo {self.params.max_jump} m, '
            f'max_range {self.params.max_range} m tras {self.params.max_range_run} lecturas')

    def on_range(self, name, msg):
        value = self.filters[name].update(float(msg.range))
        if value is None:
            return
        out = Range()
        out.header = msg.header                      # mismo stamp y frame que la ultima lectura
        out.radiation_type = msg.radiation_type
        out.field_of_view = msg.field_of_view
        out.min_range = msg.min_range
        out.max_range = msg.max_range
        out.range = float(value)
        self.out_pubs[name].publish(out)


def main(args=None):
    spin_with_cleanup(SonarFilterNode, args)


if __name__ == '__main__':
    main()
