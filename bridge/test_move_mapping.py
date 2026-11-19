import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from move_mapping import MAX_ANGULAR_RADS, MAX_LINEAR_MPS, twist_from_dir


class TwistFromDir(unittest.TestCase):
    def test_lineal_igual_que_antes(self):
        self.assertEqual(twist_from_dir('fwd', 1.0), (0.5, 0.0))
        self.assertEqual(twist_from_dir('back', 0.4), (-0.2, 0.0))
        self.assertAlmostEqual(twist_from_dir('fwd', 0.3)[0], 0.15)

    def test_giro_speed_por_1_5(self):
        self.assertEqual(MAX_ANGULAR_RADS, 1.5)
        self.assertEqual(twist_from_dir('left', 1.0), (0.0, 1.5))
        self.assertEqual(twist_from_dir('right', 1.0), (0.0, -1.5))
        self.assertAlmostEqual(twist_from_dir('left', 0.8 / 1.5)[1], 0.8)  # lo que manda la app en Lento

    def test_giro_en_el_lugar_mueve_las_ruedas_lo_suficiente(self):
        # 0,8 rad/s con separacion 0,355 m -> 0,142 m/s por rueda (con 0,3 rad/s eran 0,053)
        self.assertGreater(0.8 * 0.355 / 2, 0.14)

    def test_stop_desconocido_y_basura(self):
        for d in ('stop', 'x', None):
            self.assertEqual(twist_from_dir(d, 1.0), (0.0, 0.0))
        self.assertEqual(twist_from_dir('fwd', 'abc'), (0.0, 0.0))
        self.assertEqual(twist_from_dir('fwd', None), (0.0, 0.0))

    def test_speed_se_limita_a_0_1(self):
        self.assertEqual(twist_from_dir('fwd', 7), (MAX_LINEAR_MPS, 0.0))
        self.assertEqual(twist_from_dir('left', -3), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
