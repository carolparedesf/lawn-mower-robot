import math
import random
import unittest

from lawnbot_nav.blade_logic import (B1, B2, B3, B4, B5, BUMPERS, BladeCore, BladeInputs, BladeParams,
                                     effective_max_duty, tilt_from_quaternion)


def good(**kw):
    """Entradas con todas las condiciones cumplidas y los dos motores pedidos."""
    base = dict(request_cutter=True, request_trimmer=True, inside=True, grass=True, passes=0, tilt_deg=2.0,
                bumpers={n: False for n in BUMPERS}, estop=False)
    base.update(kw)
    return BladeInputs(**base)


def with_bumpers(*pressed):
    return {n: (n in pressed) for n in BUMPERS}


class ConditionTests(unittest.TestCase):
    def setUp(self):
        self.core = BladeCore(BladeParams(max_duty_cutter=0.3, max_duty_trimmer=0.25, max_passes=2))

    def ev(self, now=0.0, **kw):
        return self.core.evaluate(good(**kw), now)

    def test_todo_en_orden_enciende_ambos_con_su_tope(self):
        d = self.ev()
        self.assertEqual((d.cutter, d.trimmer), (0.3, 0.25))
        self.assertEqual(d.reasons, {'cutter': [], 'trimmer': []})
        self.assertEqual(d.status(), 'cutter=ON duty=0.30 | trimmer=ON duty=0.25')

    def test_sin_pedido_no_enciende_aunque_todo_este_bien(self):
        d = self.ev(request_cutter=False, request_trimmer=False)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0))
        d = self.ev(request_cutter=False)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.25))                 # cada motor se pide por separado

    def test_por_defecto_todo_apagado(self):
        d = self.core.evaluate(BladeInputs(), 0.0)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0))

    def off_both(self, d, reason):
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0), reason)
        self.assertIn(reason, d.reasons['cutter'])
        self.assertIn(reason, d.reasons['trimmer'])

    def test_fuera_del_cerco(self):
        self.off_both(self.ev(inside=False), 'fuera_del_cerco')

    def test_cerco_sin_dato(self):
        self.off_both(self.ev(inside=None), 'geofence_sin_dato')

    def test_estop(self):
        self.off_both(self.ev(estop=True), 'estop')

    def test_levantado_con_cada_par_de_bumpers(self):
        for pair in ((B3, B4), (B1, B5), (B2, B5)):
            core = BladeCore(BladeParams())
            d = core.evaluate(good(bumpers=with_bumpers(*pair)), 0.0)
            self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0), pair)
            self.assertIn('levantado', d.reasons['cutter'])

    def test_un_bumper_solo_o_otros_pares_no_son_levantado(self):
        for pressed in ((B1,), (B2,), (B3,), (B4,), (B5,), (B1, B2), (B1, B3), (B1, B4), (B2, B3), (B3, B5), (B4, B5)):
            core = BladeCore(BladeParams())
            d = core.evaluate(good(bumpers=with_bumpers(*pressed)), 0.0)
            self.assertEqual(d.cutter, 0.3, pressed)

    def test_levantado_se_mantiene_apagado_un_tiempo_tras_soltar(self):
        self.core.evaluate(good(bumpers=with_bumpers(B3, B4)), 10.0)
        d = self.ev(now=11.0)                                                  # bumpers sueltos
        self.assertIn('levantado', d.reasons['cutter'])
        self.assertEqual(d.cutter, 0.0)
        self.assertEqual(self.ev(now=13.5).cutter, 0.3)                        # lift_hold_s = 3

    def test_volcado_por_inclinacion_y_se_mantiene(self):
        d = self.ev(now=0.0, tilt_deg=45.0)
        self.off_both(d, 'volcado')
        self.assertEqual(self.ev(now=1.0, tilt_deg=5.0).cutter, 0.0)           # ya nivelado, todavia en espera
        self.assertEqual(self.ev(now=3.5, tilt_deg=5.0).cutter, 0.3)

    def test_umbral_de_inclinacion_configurable(self):
        core = BladeCore(BladeParams(max_tilt_deg=10.0))
        self.assertEqual(core.evaluate(good(tilt_deg=9.0), 0.0).cutter, 0.3)
        self.assertEqual(core.evaluate(good(tilt_deg=11.0), 1.0).cutter, 0.0)

    def test_imu_sin_dato_o_nan(self):
        self.off_both(self.ev(tilt_deg=None), 'imu_sin_dato')
        self.off_both(self.ev(tilt_deg=math.nan), 'imu_sin_dato')

    def test_bumpers_sin_dato_no_permiten_detectar_el_levantado(self):
        partial = with_bumpers()
        partial[B5] = None
        self.off_both(self.ev(bumpers=partial), 'bumpers_sin_dato')
        self.off_both(self.ev(bumpers={}), 'bumpers_sin_dato')

    def test_celda_con_demasiadas_pasadas(self):
        self.assertEqual(self.ev(passes=1).cutter, 0.3)                        # menos de 2: se corta
        self.off_both(self.ev(passes=2), 'celda_con_demasiadas_pasadas')
        self.off_both(self.ev(passes=7), 'celda_con_demasiadas_pasadas')

    def test_pasadas_desconocidas(self):
        self.off_both(self.ev(passes=None), 'pasadas_desconocidas')
        self.off_both(self.ev(passes=-1), 'pasadas_desconocidas')

    def test_el_limite_de_pasadas_es_un_parametro(self):
        core = BladeCore(BladeParams(max_passes=5))
        self.assertEqual(core.evaluate(good(passes=4), 0.0).cutter, 0.3)
        self.assertEqual(core.evaluate(good(passes=5), 0.0).cutter, 0.0)

    def test_pasto_solo_afecta_a_la_cuchilla(self):
        d = self.ev(grass=False)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.25))
        self.assertIn('sin_pasto', d.reasons['cutter'])
        self.assertEqual(d.reasons['trimmer'], [])
        d = self.ev(grass=None)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.25))
        self.assertIn('pasto_sin_dato', d.reasons['cutter'])

    def test_require_grass_false_ignora_el_pasto(self):
        core = BladeCore(BladeParams(require_grass=False))
        self.assertEqual(core.evaluate(good(grass=None), 0.0).cutter, 0.3)

    def test_el_trimmer_si_respeta_las_demas_condiciones(self):
        for kw in (dict(inside=False), dict(estop=True), dict(passes=3), dict(tilt_deg=80.0), dict(inside=None)):
            d = BladeCore(BladeParams()).evaluate(good(**kw), 0.0)
            self.assertEqual(d.trimmer, 0.0, kw)


class TiltDefaultTests(unittest.TestCase):
    def test_el_umbral_por_defecto_es_20_grados(self):
        self.assertEqual(BladeParams().max_tilt_deg, 20.0)
        core = BladeCore(BladeParams())
        self.assertEqual(core.evaluate(good(tilt_deg=19.9), 0.0).cutter, 0.3)
        self.assertEqual(core.evaluate(good(tilt_deg=20.0), 1.0).cutter, 0.3)             # exactamente en el umbral: todavia no
        self.assertEqual(core.evaluate(good(tilt_deg=20.1), 2.0).cutter, 0.0)

    def test_apaga_de_inmediato_al_superarlo(self):
        core = BladeCore(BladeParams())
        d = core.evaluate(good(tilt_deg=5.0), 100.0)
        self.assertEqual((d.cutter, d.trimmer), (0.3, 0.3))
        d = core.evaluate(good(tilt_deg=25.0), 100.1)                                       # el ciclo siguiente ya esta en 0
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0))
        self.assertIn('volcado', d.reasons['cutter'])

    def test_rearma_3_s_despues_de_volver_por_debajo_del_umbral(self):
        core = BladeCore(BladeParams())
        core.evaluate(good(tilt_deg=40.0), 10.0)                                            # volcado
        for t in (10.1, 11.0, 12.0, 12.9):                                                  # nivelado pero en espera
            self.assertEqual(core.evaluate(good(tilt_deg=3.0), t).cutter, 0.0, t)
        self.assertEqual(core.evaluate(good(tilt_deg=3.0), 13.1).cutter, 0.3)

    def test_la_espera_se_reinicia_si_vuelve_a_inclinarse(self):
        core = BladeCore(BladeParams())
        core.evaluate(good(tilt_deg=40.0), 10.0)
        core.evaluate(good(tilt_deg=3.0), 12.0)
        core.evaluate(good(tilt_deg=30.0), 12.5)                                            # rebote: otra vez inclinado
        self.assertEqual(core.evaluate(good(tilt_deg=3.0), 14.0).cutter, 0.0)               # 3 s desde 12.5
        self.assertEqual(core.evaluate(good(tilt_deg=3.0), 15.6).cutter, 0.3)


class ForceCutTests(unittest.TestCase):
    def setUp(self):
        self.core = BladeCore(BladeParams())

    def test_force_cut_ignora_el_pasto_ausente_o_sin_dato(self):
        for grass in (False, None):
            d = self.core.evaluate(good(grass=grass, force_cut=True), 0.0)
            self.assertEqual(d.cutter, 0.3, grass)
            d = self.core.evaluate(good(grass=grass, force_cut=False), 0.0)
            self.assertEqual(d.cutter, 0.0, grass)

    def test_force_cut_no_anula_ninguna_otra_condicion(self):
        casos = {
            'sin_pedido': dict(request_cutter=False),
            'fuera_del_cerco': dict(inside=False),
            'geofence_sin_dato': dict(inside=None),
            'estop': dict(estop=True),
            'volcado': dict(tilt_deg=45.0),
            'imu_sin_dato': dict(tilt_deg=None),
            'celda_con_demasiadas_pasadas': dict(passes=5),
            'pasadas_desconocidas': dict(passes=None),
            'bumpers_sin_dato': dict(bumpers={}),
        }
        for reason, kw in casos.items():
            d = BladeCore(BladeParams()).evaluate(good(grass=False, force_cut=True, **kw), 0.0)
            self.assertEqual((d.cutter, d.trimmer) if reason != 'sin_pedido' else (d.cutter,),
                             (0.0, 0.0) if reason != 'sin_pedido' else (0.0,), reason)
            self.assertIn(reason, d.reasons['cutter'], reason)

    def test_force_cut_no_anula_el_levantado(self):
        for pair in ((B3, B4), (B1, B5), (B2, B5)):
            d = BladeCore(BladeParams()).evaluate(good(grass=False, force_cut=True, bumpers=with_bumpers(*pair)), 0.0)
            self.assertEqual(d.cutter, 0.0, pair)
            self.assertIn('levantado', d.reasons['cutter'])

    def test_force_cut_sin_pedido_no_enciende_nada(self):
        d = BladeCore(BladeParams()).evaluate(good(request_cutter=False, request_trimmer=False, force_cut=True), 0.0)
        self.assertEqual((d.cutter, d.trimmer), (0.0, 0.0))

    def test_force_cut_no_cambia_al_trimmer(self):
        a = BladeCore(BladeParams()).evaluate(good(force_cut=False), 0.0)
        b = BladeCore(BladeParams()).evaluate(good(force_cut=True), 0.0)
        self.assertEqual((a.trimmer, a.reasons['trimmer']), (b.trimmer, b.reasons['trimmer']))

    def test_force_cut_respeta_el_tope_de_duty(self):
        d = BladeCore(BladeParams(max_duty_cutter=0.2)).evaluate(good(grass=False, force_cut=True), 0.0)
        self.assertEqual(d.cutter, 0.2)

    def test_propiedad_force_cut_solo_cambia_el_resultado_por_el_pasto(self):
        rng = random.Random(9)
        params = BladeParams(max_passes=3, max_tilt_deg=25.0)
        for _ in range(5000):
            kw = dict(request_cutter=rng.random() < 0.8, request_trimmer=rng.random() < 0.8,
                      inside=rng.choice([True, True, False, None]), grass=rng.choice([True, False, None]),
                      passes=rng.choice([None, -1, 0, 2, 3, 4]), tilt_deg=rng.choice([None, 0.0, 24.0, 26.0, 90.0]),
                      bumpers={n: rng.choice([True, False, False, None]) for n in BUMPERS}, estop=rng.random() < 0.1)
            off = BladeCore(params).evaluate(BladeInputs(force_cut=False, **kw), 0.0)
            on = BladeCore(params).evaluate(BladeInputs(force_cut=True, **kw), 0.0)
            self.assertEqual(off.trimmer, on.trimmer)                                      # el trimmer no cambia nunca
            if on.cutter > 0.0 and off.cutter == 0.0:                                      # lo unico que force_cut agrega
                self.assertTrue(set(off.reasons['cutter']) <= {'sin_pasto', 'pasto_sin_dato'})
            self.assertGreaterEqual(on.cutter, off.cutter)


class DutyCapTests(unittest.TestCase):
    def test_el_tope_por_defecto_es_conservador(self):
        p = BladeParams()
        self.assertLessEqual(p.max_duty_cutter, 0.3)
        self.assertLessEqual(p.max_duty_trimmer, 0.3)

    def test_el_tope_se_limita_con_las_tensiones(self):
        self.assertAlmostEqual(effective_max_duty(0.9, 12.0, 24.0), 0.5)       # motor de 12 V en un pack de 24 V
        self.assertAlmostEqual(effective_max_duty(0.3, 12.0, 24.0), 0.3)       # el configurado ya era menor
        self.assertEqual(effective_max_duty(0.9, 0.0, 24.0), 0.9)              # tension desconocida: no limita
        self.assertEqual(effective_max_duty(0.9, 12.0, 0.0), 0.9)
        self.assertEqual(effective_max_duty(0.5, 24.0, 12.0), 0.5)             # motor para mas tension que el pack

    def test_el_tope_siempre_esta_en_0_1(self):
        self.assertEqual(effective_max_duty(5.0, 0.0, 0.0), 1.0)
        self.assertEqual(effective_max_duty(-1.0, 12.0, 24.0), 0.0)

    def test_nunca_se_publica_mas_que_el_tope_por_motor(self):
        core = BladeCore(BladeParams(max_duty_cutter=0.2, max_duty_trimmer=0.1, pack_voltage=24.0,
                                     cutter_nominal_voltage=12.0, trimmer_nominal_voltage=6.0))
        d = core.evaluate(good(), 0.0)
        self.assertEqual((d.cutter, d.trimmer), (0.2, 0.1))
        core = BladeCore(BladeParams(max_duty_cutter=1.0, max_duty_trimmer=1.0, pack_voltage=24.0,
                                     cutter_nominal_voltage=12.0, trimmer_nominal_voltage=6.0))
        d = core.evaluate(good(), 0.0)
        self.assertEqual((d.cutter, d.trimmer), (0.5, 0.25))


class TiltTests(unittest.TestCase):
    def test_cuaternion_de_inclinacion(self):
        self.assertAlmostEqual(tilt_from_quaternion(0, 0, 0, 1), 0.0)
        self.assertAlmostEqual(tilt_from_quaternion(0, 0, math.sin(1.0), math.cos(1.0)), 0.0)        # solo yaw: no inclina
        half = math.radians(40.0) / 2
        self.assertAlmostEqual(tilt_from_quaternion(math.sin(half), 0, 0, math.cos(half)), 40.0)     # roll
        self.assertAlmostEqual(tilt_from_quaternion(0, math.sin(half), 0, math.cos(half)), 40.0)     # pitch
        self.assertAlmostEqual(tilt_from_quaternion(1, 0, 0, 0), 180.0)                              # boca abajo
        self.assertTrue(math.isnan(tilt_from_quaternion(0, 0, 0, 0)))


class SafetyPropertyTests(unittest.TestCase):
    def test_duty_positivo_solo_si_todas_las_condiciones_se_cumplen(self):
        rng = random.Random(5)
        params = BladeParams(max_duty_cutter=0.3, max_duty_trimmer=0.2, max_passes=3, max_tilt_deg=25.0)
        for _ in range(6000):
            core = BladeCore(params)
            bumpers = {n: rng.choice([True, False, False, None]) for n in BUMPERS}
            inp = BladeInputs(request_cutter=rng.random() < 0.8, request_trimmer=rng.random() < 0.8,
                              inside=rng.choice([True, True, False, None]),
                              grass=rng.choice([True, True, False, None]),
                              passes=rng.choice([None, -1, 0, 1, 2, 3, 4]),
                              tilt_deg=rng.choice([None, math.nan, 0.0, 10.0, 24.9, 25.1, 90.0]),
                              bumpers=bumpers, estop=rng.random() < 0.1)
            d = core.evaluate(inp, 0.0)
            self.assertLessEqual(d.cutter, 0.3 + 1e-12)
            self.assertLessEqual(d.trimmer, 0.2 + 1e-12)
            lifted = any(bumpers[a] and bumpers[b] for a, b in ((B3, B4), (B1, B5), (B2, B5)))
            common_ok = (not inp.estop and inp.inside is True and None not in bumpers.values() and not lifted
                         and inp.tilt_deg is not None and not math.isnan(inp.tilt_deg) and inp.tilt_deg <= 25.0
                         and inp.passes is not None and 0 <= inp.passes < 3)
            self.assertEqual(d.cutter > 0.0, common_ok and inp.request_cutter and inp.grass is True)
            self.assertEqual(d.trimmer > 0.0, common_ok and inp.request_trimmer)
            self.assertEqual(d.cutter == 0.0, bool(d.reasons['cutter']))
            self.assertEqual(d.trimmer == 0.0, bool(d.reasons['trimmer']))


if __name__ == '__main__':
    unittest.main()
