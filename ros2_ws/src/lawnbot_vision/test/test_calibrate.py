"""Calibracion: eleccion del objeto, resumen y el bloque que se pega en vision.yaml (sin camara ni modelo)."""
import os
import unittest

from lawnbot_vision.calibrate import build_result, load_params, pick_target, summarize, yaml_block
from lawnbot_vision.vision_logic import VisionCore, VisionParams

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'vision.yaml')
OBST = ['arbol', 'planta', 'barrera', 'otro']


class Target(unittest.TestCase):
    def test_la_mascara_mas_grande_entre_las_clases_pedidas(self):
        cands = [('tierra', 300, 470, 90000), ('otro', 100, 300, 500), ('planta', 320, 350, 4000)]
        self.assertEqual(pick_target(cands, OBST), ('planta', 320, 350, 4000))   # el piso (tierra) no cuenta

    def test_ninguno(self):
        self.assertIsNone(pick_target([('pasto', 1, 2, 3)], OBST))


class Result(unittest.TestCase):
    def test_mediana_y_dispersion(self):
        self.assertEqual(summarize([300, 310, 305, 400]), (307.5, 100.0))

    def test_ordena_de_lejos_a_cerca(self):
        y, d, err = build_result([(412.4, 1.0), (330.0, 2.0), (460.0, 0.5)])
        self.assertIsNone(err)
        self.assertEqual((y, d), ([330.0, 412.0, 460.0], [2.0, 1.0, 0.5]))

    def test_rechaza_mediciones_incoherentes(self):
        _, _, err = build_result([(400.0, 2.0), (330.0, 0.5)])   # mas lejos y mas abajo en la imagen
        self.assertIsNotNone(err)


class Block(unittest.TestCase):
    def pasted(self, block):
        """vision.yaml con el bloque pegado al final de ros__parameters (las claves repetidas: gana la ultima)."""
        import tempfile
        text = open(CONFIG).read().rstrip('\n') + '\n' + block + '\n'
        with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
            f.write(text)
        try:
            return load_params(f.name)
        finally:
            os.unlink(f.name)

    def test_el_bloque_pegado_en_vision_yaml_lo_acepta_el_nodo(self):
        y, d, _ = build_result([(330.0, 2.0), (412.0, 1.0), (460.0, 0.5)])
        p = self.pasted(yaml_block(y, d, 640, 480, (0.21, 0.0, 0.0)))
        self.assertIs(p['calibrated'], True)
        self.assertEqual(p['calib_y'], [330.0, 412.0, 460.0])
        self.assertEqual(p['calib_d'], [2.0, 1.0, 0.5])
        self.assertEqual(p['cam_x'], 0.21)
        for k in ('calib_y', 'calib_d', 'cam_x', 'cam_y', 'cam_yaw_deg'):
            self.assertIsInstance(p[k] if not isinstance(p[k], list) else p[k][0], float, k)   # ROS exige double
        core = VisionCore(VisionParams(**{k: p[k] for k in ('calibrated', 'calib_y', 'calib_d', 'cam_x')}))
        self.assertTrue(core.p.calibrated)

    def test_sin_montaje_no_toca_cam(self):
        p = self.pasted(yaml_block([300.0, 420.0], [1.5, 0.5], 640, 480))
        self.assertEqual(p['cam_x'], load_params(CONFIG)['cam_x'])   # queda el del YAML


if __name__ == '__main__':
    unittest.main()
