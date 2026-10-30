import json
import math
import os
import random
import tempfile
import unittest

from lawnbot_nav.geofence_logic import (Hysteresis, distance_to_boundary, drop_closing_vertex, polygon_centroid,
                                        lat_lon_to_local_xy, load_geofence, parse_geofence_json,
                                        point_in_polygon, save_geofence, segments_intersect,
                                        signed_area, signed_distance, validate_polygon)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]


class ParseTests(unittest.TestCase):
    def test_formato_de_la_app(self):
        text = json.dumps({'points': [{'lat': -25.3, 'lng': -57.6}, {'lat': -25.31, 'lng': -57.6}]})
        self.assertEqual(parse_geofence_json(text), [(-25.3, -57.6), (-25.31, -57.6)])

    def test_acepta_lon(self):
        self.assertEqual(parse_geofence_json('{"points":[{"lat":1,"lon":2}]}'), [(1.0, 2.0)])

    def test_rechaza_entradas_invalidas(self):
        for bad in ['no es json', '[]', '{"points": 3}', '{"points":[1]}',
                    '{"points":[{"lat":"a","lng":1}]}', '{"points":[{"lat":95,"lng":1}]}',
                    '{"points":[{"lat":1}]}', '{"points":[{"lat":true,"lng":1}]}']:
            with self.assertRaises(ValueError, msg=bad):
                parse_geofence_json(bad)

    def test_quita_el_vertice_de_cierre(self):
        closed = SQUARE + [SQUARE[0]]
        self.assertEqual(drop_closing_vertex(closed), SQUARE)
        self.assertEqual(drop_closing_vertex(SQUARE), SQUARE)


class ValidateTests(unittest.TestCase):
    def test_cuadrado_valido_y_antihorario(self):
        poly, err = validate_polygon(SQUARE)
        self.assertIsNone(err)
        self.assertGreater(signed_area(poly), 0)

    def test_sentido_horario_se_normaliza(self):
        poly, err = validate_polygon(list(reversed(SQUARE)))
        self.assertIsNone(err)
        self.assertGreater(signed_area(poly), 0)

    def test_acepta_el_cierre_repetido(self):
        poly, err = validate_polygon(SQUARE + [SQUARE[0]])
        self.assertIsNone(err)
        self.assertEqual(len(poly), 4)

    def test_minimo_de_vertices(self):
        poly, err = validate_polygon([(0, 0), (10, 0), (5, 8)])
        self.assertIsNone(poly)
        self.assertIn('vertices', err)

    def test_autointerseccion_moño(self):
        poly, err = validate_polygon([(0, 0), (10, 10), (10, 0), (0, 10)])
        self.assertIsNone(poly)
        self.assertIn('autointerseccion', err)

    def test_vertice_que_roza_otra_arista(self):
        # el vertice 5 toca la arista inferior
        pts = [(0, 0), (10, 0), (10, 10), (5, 0), (0, 10), (0, 5)]
        poly, err = validate_polygon(pts)
        self.assertIsNone(poly)

    def test_arista_demasiado_corta(self):
        poly, err = validate_polygon([(0, 0), (10, 0), (10.1, 0.1), (10, 10), (0, 10)])
        self.assertIsNone(poly)
        self.assertIn('arista', err)

    def test_vertices_duplicados(self):
        poly, err = validate_polygon([(0, 0), (10, 0), (10, 0), (10, 10), (0, 10)])
        self.assertIsNone(poly)

    def test_arista_que_vuelve_sobre_si_misma(self):
        poly, err = validate_polygon([(0, 0), (10, 0), (5, 0), (5, 10), (0, 10)])
        self.assertIsNone(poly)

    def test_area_minima(self):
        poly, err = validate_polygon([(0, 0), (1, 0), (1, 1), (0, 1)], min_edge=0.5, min_area=2.0)
        self.assertIsNone(poly)
        self.assertIn('area', err)

    def test_poligono_concavo_simple_es_valido(self):
        ele = [(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)]
        poly, err = validate_polygon(ele)
        self.assertIsNone(err)

    def test_interseccion_de_segmentos(self):
        self.assertTrue(segments_intersect((0, 0), (4, 4), (0, 4), (4, 0)))
        self.assertTrue(segments_intersect((0, 0), (4, 0), (4, 0), (4, 4)))      # roce en extremo
        self.assertTrue(segments_intersect((0, 0), (4, 0), (2, 0), (6, 0)))      # colineal solapado
        self.assertFalse(segments_intersect((0, 0), (4, 0), (5, 0), (9, 0)))
        self.assertFalse(segments_intersect((0, 0), (4, 0), (0, 1), (4, 1)))


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.poly, _ = validate_polygon(SQUARE)

    def test_punto_en_poligono(self):
        self.assertTrue(point_in_polygon((5, 5), self.poly))
        self.assertFalse(point_in_polygon((11, 5), self.poly))
        self.assertFalse(point_in_polygon((-0.1, 5), self.poly))
        self.assertTrue(point_in_polygon((10, 5), self.poly))     # en el borde
        self.assertTrue(point_in_polygon((0, 0), self.poly))      # en un vertice

    def test_distancia_con_signo(self):
        self.assertAlmostEqual(signed_distance((5, 5), self.poly), 5.0)
        self.assertAlmostEqual(signed_distance((9, 5), self.poly), 1.0)
        self.assertAlmostEqual(signed_distance((12, 5), self.poly), -2.0)
        self.assertAlmostEqual(signed_distance((13, 14), self.poly), -5.0)    # cerca de un vertice
        self.assertEqual(signed_distance((10, 5), self.poly), 0.0)

    def test_concavo(self):
        ele, _ = validate_polygon([(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)])
        self.assertTrue(point_in_polygon((2, 8), ele))
        self.assertFalse(point_in_polygon((7, 7), ele))            # en la muesca
        self.assertAlmostEqual(signed_distance((7, 7), ele), -3.0)

    def test_distancia_a_un_punto_degenerado(self):
        self.assertAlmostEqual(distance_to_boundary((3, 4), [(0, 0), (0, 0), (0, 0)]), 5.0)

    def test_centroide(self):
        self.assertEqual(polygon_centroid(self.poly), (5.0, 5.0))
        ele, _ = validate_polygon([(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)])
        cx, cy = polygon_centroid(ele)
        self.assertAlmostEqual(cx, cy)                                  # la L es simetrica respecto de y = x
        self.assertAlmostEqual(cx, 3.875)
        self.assertEqual(polygon_centroid([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]), (2.0, 0.0))   # degenerado

    def test_conversion_local(self):
        x, y = lat_lon_to_local_xy(-25.3, -57.6, -25.3, -57.6)
        self.assertEqual((x, y), (0.0, 0.0))
        _, north = lat_lon_to_local_xy(-25.299, -57.6, -25.3, -57.6)
        self.assertAlmostEqual(north, 110.9, delta=0.5)              # 0,001 grado de latitud ~ 111 m
        east, _ = lat_lon_to_local_xy(-25.3, -57.599, -25.3, -57.6)
        self.assertAlmostEqual(east, 100.4, delta=0.5)               # 0,001 grado de longitud a 25 S


class HysteresisTests(unittest.TestCase):
    def test_no_parpadea_con_lecturas_alternadas(self):
        h = Hysteresis(to_true=5, to_false=2)
        h.state = True
        out = [h.update(v) for v in [False, True, False, True, False, True]]
        self.assertTrue(all(out))

    def test_cambia_tras_n_lecturas_consecutivas(self):
        h = Hysteresis(to_true=5, to_false=2)
        self.assertEqual([h.update(True) for _ in range(5)], [False, False, False, False, True])
        self.assertEqual([h.update(False) for _ in range(2)], [True, False])

    def test_desconocido_fuerza_false_de_inmediato(self):
        h = Hysteresis(to_true=5, to_false=2, initial=True)
        self.assertFalse(h.update(None))
        self.assertFalse(h.update(True))          # y vuelve a exigir las N lecturas
        self.assertEqual(sum(h.update(True) for _ in range(5)), 2)

    def crossing_changes(self, to_true, to_false, sigma, seed=3):
        """Cambios de estado al cruzar despacio (5 cm/s) el borde derecho con ruido de GPS."""
        poly, _ = validate_polygon(SQUARE)
        rng = random.Random(seed)
        h = Hysteresis(to_true, to_false)
        raw_changes = filtered_changes = 0
        last_raw = last_filtered = None
        for i in range(1200):                      # 120 s a 10 Hz
            x = 9.7 + 0.005 * i
            measured = (x + rng.gauss(0, sigma), 5.0 + rng.gauss(0, sigma))
            raw = signed_distance(measured, poly) >= 0
            filtered = h.update(raw)
            if last_raw is not None:
                raw_changes += raw != last_raw
                filtered_changes += filtered != last_filtered
            last_raw, last_filtered = raw, filtered
        return raw_changes, filtered_changes, h.state

    def test_robot_cruzando_el_borde_con_ruido_de_gps(self):
        for sigma in (0.05, 0.15, 0.4):
            raw, filtered, final = self.crossing_changes(5, 2, sigma)
            self.assertGreater(raw, 15, f'sigma {sigma}')      # sin histeresis parpadea mucho
            self.assertLessEqual(filtered, 4, f'sigma {sigma}')
            self.assertFalse(final)                            # termina afuera

    def test_mas_lecturas_consecutivas_dan_menos_cambios(self):
        _, few, _ = self.crossing_changes(5, 2, 0.4)
        _, many, _ = self.crossing_changes(10, 5, 0.4)
        self.assertLessEqual(many, few)


class PersistenceTests(unittest.TestCase):
    def test_guardar_y_cargar_en_lat_lon(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'sub', 'cerco.json')
            pts = [(-25.3, -57.6), (-25.3, -57.599), (-25.299, -57.599), (-25.299, -57.6)]
            save_geofence(path, pts)
            self.assertEqual(load_geofence(path), pts)

    def test_archivo_inexistente_o_corrupto(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(load_geofence(os.path.join(d, 'no_existe.json')))
            bad = os.path.join(d, 'malo.json')
            with open(bad, 'w') as f:
                f.write('{no es json')
            self.assertIsNone(load_geofence(bad))


if __name__ == '__main__':
    unittest.main()
