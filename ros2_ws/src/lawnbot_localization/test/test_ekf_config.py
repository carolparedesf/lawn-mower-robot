"""Los EKF toman vx de la odometria de ruedas y el giro (vyaw) del IMU: el rumbo no depende del ancho de via."""
import os
import unittest

import yaml

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
NAMES = ['x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az']


def load(node):
    with open(os.path.join(CONFIG, f'{node}.yaml')) as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def fused(params, key):
    cfg = params[f'{key}_config']
    assert len(cfg) == 15, key
    return {n for n, on in zip(NAMES, cfg) if on}


class EkfConfigTests(unittest.TestCase):
    def test_la_odometria_de_ruedas_solo_aporta_vx(self):
        for node in ('ekf_local', 'ekf_global'):
            p = load(node)
            self.assertEqual(p['odom0'], '/odom/unfiltered')
            self.assertEqual(fused(p, 'odom0'), {'vx'}, node)

    def test_el_giro_lo_aporta_el_imu(self):
        for node in ('ekf_local', 'ekf_global'):
            p = load(node)
            self.assertEqual(p['imu0'], '/imu/data')
            self.assertIn('vyaw', fused(p, 'imu0'), node)
        # ningun otro sensor aporta vyaw
        for node in ('ekf_local', 'ekf_global'):
            p = load(node)
            others = [k for k in ('odom0', 'odom1') if k in p]
            self.assertTrue(all('vyaw' not in fused(p, k) for k in others), node)

    def test_ekf_local_sin_yaw_absoluto(self):
        self.assertEqual(fused(load('ekf_local'), 'imu0'), {'vyaw'})

    def test_ekf_global_sigue_con_yaw_absoluto_del_imu_y_el_gps(self):
        p = load('ekf_global')
        self.assertEqual(fused(p, 'imu0'), {'yaw', 'vyaw'})
        self.assertEqual(p['odom1'], '/odometry/gps')
        self.assertEqual(fused(p, 'odom1'), {'x', 'y'})

    def test_el_resto_de_la_configuracion_no_cambio(self):
        for node, world in (('ekf_local', 'odom'), ('ekf_global', 'map')):
            p = load(node)
            self.assertEqual((p['frequency'], p['sensor_timeout'], p['two_d_mode'], p['world_frame']), (30.0, 0.2, True, world))
            self.assertFalse(p['odom0_differential'] or p['odom0_relative'] or p['imu0_differential'] or p['imu0_relative'])


if __name__ == '__main__':
    unittest.main()
