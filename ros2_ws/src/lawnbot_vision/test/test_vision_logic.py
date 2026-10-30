import math
import unittest

from lawnbot_vision.vision_logic import (Detection, VisionCore, VisionParams, base_point, bearing_from_x,
                                         distance_from_y, to_base_footprint, validate_calibration)

CAL_Y, CAL_D = [300.0, 420.0], [1.5, 0.5]


def core(**kw):
    base = dict(calibrated=True, calib_y=CAL_Y, calib_d=CAL_D, fov_h_deg=62.0, grass_window=7)
    base.update(kw)
    return VisionCore(VisionParams(**base))


class Distance(unittest.TestCase):
    def test_interpola_entre_los_puntos_calibrados(self):
        self.assertAlmostEqual(distance_from_y(360.0, CAL_Y, CAL_D), 1.0)
        self.assertAlmostEqual(distance_from_y(300.0, CAL_Y, CAL_D), 1.5)
        self.assertAlmostEqual(distance_from_y(420.0, CAL_Y, CAL_D), 0.5)

    def test_mas_lejos_que_lo_calibrado_es_desconocido(self):
        self.assertIsNone(distance_from_y(250.0, CAL_Y, CAL_D))

    def test_mas_cerca_que_lo_calibrado_usa_la_mas_cercana(self):
        self.assertEqual(distance_from_y(470.0, CAL_Y, CAL_D), 0.5)

    def test_cuatro_puntos(self):
        y, d = [200.0, 300.0, 380.0, 440.0], [2.0, 1.5, 1.0, 0.5]
        self.assertAlmostEqual(distance_from_y(340.0, y, d), 1.25)
        self.assertAlmostEqual(distance_from_y(410.0, y, d), 0.75)


class Calibration(unittest.TestCase):
    def test_validas(self):
        self.assertIsNone(validate_calibration(CAL_Y, CAL_D))

    def test_invalidas(self):
        self.assertIsNotNone(validate_calibration([420.0, 300.0], CAL_D))        # y decrece
        self.assertIsNotNone(validate_calibration(CAL_Y, [0.5, 1.5]))            # d crece
        self.assertIsNotNone(validate_calibration([300.0], [1.5]))               # un solo punto
        self.assertIsNotNone(validate_calibration(CAL_Y, [1.5, 0.5, 0.2]))       # largos distintos
        self.assertIsNotNone(validate_calibration(CAL_Y, [1.5, -0.5]))           # distancia no positiva

    def test_el_nucleo_no_arranca_con_calibracion_invalida(self):
        with self.assertRaises(ValueError):
            VisionCore(VisionParams(calib_y=[420.0, 300.0]))


class Geometry(unittest.TestCase):
    def test_angulo_positivo_a_la_derecha(self):
        self.assertAlmostEqual(bearing_from_x(320, 640, 62.0), 0.0)
        self.assertAlmostEqual(bearing_from_x(640, 640, 62.0), math.radians(31.0))
        self.assertAlmostEqual(bearing_from_x(0, 640, 62.0), -math.radians(31.0))

    def test_obstaculo_a_la_derecha_queda_con_y_negativa_en_base_footprint(self):
        x, y = to_base_footprint(math.radians(20), 1.0, 0.0, 0.0, 0.0)
        self.assertGreater(x, 0.0)
        self.assertLess(y, 0.0, 'y positiva es izquierda: a la derecha es negativa')

    def test_montaje_de_la_camara(self):
        x, y = to_base_footprint(0.0, 1.0, 0.2, 0.05, 0.0)
        self.assertAlmostEqual(x, 1.2)
        self.assertAlmostEqual(y, 0.05)
        x, y = to_base_footprint(0.0, 1.0, 0.0, 0.0, math.radians(90))    # camara mirando a la izquierda
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, 1.0)

    def test_base_point_es_la_fila_mas_baja(self):
        self.assertEqual(base_point([(10, 5), (20, 40), (30, 40), (99, 10)]), (25, 40))


class Obstacles(unittest.TestCase):
    def test_sin_calibrar_no_publica_obstaculos_pero_si_pasto(self):
        c = core(calibrated=False)
        r = c.process(0.5, [Detection('arbol', 320, 360)], 640)
        self.assertEqual(r.obstacles, [])
        self.assertTrue(r.grass)

    def test_obstaculo_al_frente_a_un_metro(self):
        r = core().process(0.5, [Detection('arbol', 320, 360)], 640)
        self.assertEqual(len(r.obstacles), 1)
        x, y = r.obstacles[0]
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)

    def test_clases_que_no_son_obstaculo_se_ignoran(self):
        r = core().process(0.5, [Detection('caminero', 320, 360), Detection('tierra', 320, 360)], 640)
        self.assertEqual(r.obstacles, [])

    def test_mas_lejos_que_el_rango_calibrado_se_cuenta_y_no_se_publica(self):
        r = core().process(0.5, [Detection('planta', 320, 200)], 640)
        self.assertEqual(r.obstacles, [])
        self.assertEqual(r.skipped_far, 1)

    def test_con_montaje(self):
        r = core(cam_x=0.2).process(0.5, [Detection('barrera', 320, 420)], 640)
        self.assertAlmostEqual(r.obstacles[0][0], 0.7)


class Grass(unittest.TestCase):
    def test_umbral_de_fraccion(self):
        c = core(min_grass=0.10, grass_window=1)
        self.assertFalse(c.process(0.09, [], 640).grass)
        self.assertTrue(c.process(0.10, [], 640).grass)

    def test_mayoria_de_la_ventana(self):
        c = core(grass_window=7)
        for f in (0.5, 0.5, 0.5, 0.5, 0.0, 0.0, 0.0):
            r = c.process(f, [], 640)
        self.assertTrue(r.grass, '4 de 7 es mayoria')
        c2 = core(grass_window=7)
        for f in (0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0):
            r = c2.process(f, [], 640)
        self.assertFalse(r.grass, '3 de 7 no es mayoria')

    def test_un_cuadro_suelto_sin_pasto_no_lo_apaga(self):
        c = core(grass_window=7)
        for _ in range(6):
            c.process(0.5, [], 640)
        self.assertTrue(c.process(0.0, [], 640).grass)

    def test_reset_olvida_la_historia(self):
        c = core(grass_window=7)
        for _ in range(7):
            c.process(0.5, [], 640)
        c.reset()
        self.assertFalse(c.process(0.0, [], 640).grass)


if __name__ == '__main__':
    unittest.main()
