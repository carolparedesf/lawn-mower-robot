import json
import math
import unittest

from lawnbot_nav.coverage_map_logic import CoverageGrid, decode_layer

FENCE = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]


def grid(**kw):
    return CoverageGrid.from_polygon(FENCE, 0.5, pad=1.0, **kw)


def drive_line(g, x_from, x_to, y, cutting=True, step=0.05):
    n = int(abs(x_to - x_from) / step)
    sign = 1 if x_to > x_from else -1
    last = None
    for k in range(n + 1):
        last = g.update_pose(x_from + sign * k * step, y, cutting)
    return last


class GridTests(unittest.TestCase):
    def test_tamano_y_limites(self):
        g = grid()
        self.assertEqual((g.x0, g.y0), (-1.0, -1.0))
        self.assertEqual((g.w, g.h), (24, 16))                      # (10+2)/0.5 x (6+2)/0.5
        self.assertEqual(g.cell_of(-1.0, -1.0), (0, 0))
        self.assertEqual(g.cell_of(0.0, 0.0), (2, 2))
        self.assertEqual(g.cell_of(10.99, 6.99), (23, 15))
        self.assertIsNone(g.cell_of(11.0, 3.0))
        self.assertIsNone(g.cell_of(-1.01, 3.0))

    def test_centro_de_celda(self):
        g = grid()
        self.assertEqual(g.center((2, 2)), (0.25, 0.25))


class PassesTests(unittest.TestCase):
    def test_primera_visita_no_cuenta_la_visita_en_curso(self):
        g = grid()
        self.assertEqual(g.update_pose(1.2, 1.2, True), 0)
        self.assertEqual(g.update_pose(1.3, 1.2, True), 0)

    def test_la_pasada_se_cuenta_al_salir_de_la_celda(self):
        g = grid()
        g.update_pose(1.1, 1.1, True)
        self.assertEqual(sum(g.visits), 0)
        drive_line(g, 1.1, 3.0, 1.1)                                  # sale de la celda y recorre varias
        self.assertGreaterEqual(g.visits[g.idx(g.cell_of(1.1, 1.1))], 1)

    def test_pasadas_sucesivas_en_el_mismo_pasillo(self):
        g = grid()
        counts = []
        for k in range(3):
            a, b = (1.0, 8.0) if k % 2 == 0 else (8.0, 1.0)
            drive_line(g, a, b, 3.0)
            counts.append(g.visits[g.idx(g.cell_of(4.0, 3.0))])
        # lo completado de una celda del medio tras cada recorrido
        self.assertEqual(counts, [1, 2, 3])

    def test_lo_que_ve_el_cortador_al_entrar_es_lo_ya_completado(self):
        g = grid()
        drive_line(g, 1.0, 8.0, 3.0)
        drive_line(g, 8.0, 1.0, 3.0)
        seen = []
        for k in range(0, 71):
            seen.append(g.update_pose(1.0 + k * 0.1, 3.0, True))
        self.assertEqual(set(seen[10:60]), {2})                       # 2 pasadas completadas: la 3ra ya no deberia cortar

    def test_sin_cortar_no_cuenta_pasadas(self):
        g = grid()
        drive_line(g, 1.0, 8.0, 3.0, cutting=False)
        self.assertEqual(sum(g.visits), 0)

    def test_cortar_solo_en_parte_de_la_visita_cuenta(self):
        g = grid()
        g.update_pose(1.1, 3.1, False)
        g.update_pose(1.2, 3.1, True)
        g.update_pose(1.3, 3.1, False)
        g.update_pose(3.0, 3.1, False)                                # sale
        self.assertEqual(g.visits[g.idx(g.cell_of(1.1, 3.1))], 1)

    def test_el_vaiven_en_el_borde_de_la_celda_no_suma_pasadas(self):
        g = grid()
        g.update_pose(1.2, 3.2, True)                                 # celda x [1.0, 1.5)
        for k in range(200):                                          # oscila +-3 cm alrededor del borde x = 1.5
            g.update_pose(1.5 + (0.03 if k % 2 else -0.03), 3.2, True)
        self.assertEqual(max(g.visits), 0)

    def test_fuera_de_la_grilla_es_desconocido(self):
        g = grid()
        self.assertIsNone(g.update_pose(50.0, 50.0, True))
        self.assertIsNone(g.update_pose(-5.0, 0.0, True))

    def test_al_salir_de_la_grilla_se_cierra_la_visita(self):
        g = grid()
        g.update_pose(1.1, 3.1, True)
        self.assertIsNone(g.update_pose(50.0, 3.1, True))
        self.assertEqual(g.visits[g.idx(g.cell_of(1.1, 3.1))], 1)

    def test_el_conteo_se_satura_en_255(self):
        g = grid()
        for _ in range(300):
            g.update_pose(1.1, 3.1, True)
            g.update_pose(50.0, 3.1, True)
        self.assertEqual(g.visits[g.idx(g.cell_of(1.1, 3.1))], 255)


class ObstacleTests(unittest.TestCase):
    def test_un_eco_sube_la_certeza_de_la_celda_del_obstaculo(self):
        g = grid()
        g.add_sonar(2.0, 3.0, 0.0, 1.0, 1.5)                          # pared a 1.0 m hacia +x: celda de (3.0, 3.0)
        k = g.idx(g.cell_of(3.0, 3.0))
        self.assertAlmostEqual(g.sonar[k], 0.4)
        for _ in range(10):
            g.add_sonar(2.0, 3.0, 0.0, 1.0, 1.5)
        self.assertGreater(g.sonar[k], 0.95)

    def test_el_espacio_libre_antes_del_eco_baja_la_certeza(self):
        g = grid()
        k_mid = g.idx(g.cell_of(2.5, 3.0))
        g.sonar[k_mid] = 0.8
        g.add_sonar(2.0, 3.0, 0.0, 1.0, 1.5)
        self.assertLess(g.sonar[k_mid], 0.8)

    def test_sin_eco_solo_limpia(self):
        g = grid()
        k = g.idx(g.cell_of(3.0, 3.0))
        g.sonar[k] = 0.9
        g.add_sonar(2.0, 3.0, 0.0, 1.5, 1.5)                          # lectura = max_range: no hay nada
        self.assertLess(g.sonar[k], 0.9)
        self.assertEqual(max(g.sonar[i] for i in range(len(g.sonar)) if i != k), 0.0)

    def test_camara_y_sonar_son_capas_separadas(self):
        g = grid()
        g.add_camera([(5.0, 3.0)])
        k = g.idx(g.cell_of(5.0, 3.0))
        self.assertEqual((g.camera[k], g.sonar[k]), (0.5, 0.0))
        g.add_camera([(5.0, 3.0), (5.0, 3.0)])
        self.assertGreater(g.camera[k], 0.8)

    def test_obstaculos_fuera_de_la_grilla_se_ignoran(self):
        g = grid()
        g.add_camera([(100.0, 100.0)])
        g.add_sonar(100.0, 100.0, 0.0, 0.5, 1.5)
        self.assertEqual(max(g.camera), 0.0)


class OutputTests(unittest.TestCase):
    def test_mensaje_codificado_ida_y_vuelta(self):
        g = grid()
        drive_line(g, 1.0, 8.0, 3.0)
        g.update_pose(50.0, 0.0, True)
        g.add_camera([(5.0, 3.0)])
        g.add_sonar(2.0, 3.0, 0.0, 1.0, 1.5)
        msg = json.loads(json.dumps(g.to_dict((2.0, 3.0, 0.5), stamp=12.5)))      # debe ser serializable
        self.assertEqual((msg['width'], msg['height'], msg['resolution']), (g.w, g.h, 0.5))
        self.assertEqual(msg['origin'], [-1.0, -1.0])
        self.assertEqual(msg['robot'], [2.0, 3.0, 0.5])
        passes = decode_layer(msg['passes'])
        self.assertEqual(len(passes), g.w * g.h)
        self.assertEqual(passes[g.idx(g.cell_of(4.0, 3.0))], 1)
        self.assertEqual(decode_layer(msg['obstacles_camera'])[g.idx(g.cell_of(5.0, 3.0))], 50)
        self.assertEqual(decode_layer(msg['obstacles_sonar'])[g.idx(g.cell_of(3.0, 3.0))], 40)

    def test_tamano_del_mensaje_de_un_cerco_grande(self):
        g = CoverageGrid.from_polygon([(0, 0), (40, 0), (40, 30), (0, 30)], 0.25, pad=1.0)
        size = len(json.dumps(g.to_dict()))
        self.assertLess(size, 120_000)                                 # 42 x 32 m a 0.25 m: ~100 KB en base64


if __name__ == '__main__':
    unittest.main()
