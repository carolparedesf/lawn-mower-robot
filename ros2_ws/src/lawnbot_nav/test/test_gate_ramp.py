import random
import statistics
import unittest

from lawnbot_nav.safety_logic import SafetyParams, SafetyState, SonarLimiter, apply_safety
from lawnbot_nav.sonar_filter_logic import SonarFilter

DT = 0.05            # el gate corre a 20 Hz
SONAR_DT = 0.2       # los sonares publican a 5 Hz


def gate_step(limiter, t, v, readings, params=None, **kw):
    return apply_safety(v, 0.0, SafetyState(sonar_ranges=readings, **kw), params or SafetyParams(),
                        limiter=limiter, now=t)


class Hysteresis(unittest.TestCase):
    def setUp(self):
        self.p = SafetyParams()                     # slow 0.30, stop 0.15, histeresis 0.05
        self.lim = SonarLimiter()
        self.t = 0.0

    def go(self, d, v=0.3):
        self.t += DT
        return gate_step(self.lim, self.t, v, {'center': d}, self.p)

    def test_entra_a_la_parada_y_sale_recien_sobre_stop_mas_histeresis(self):
        self.assertGreater(self.go(0.16)[0], 0.0)           # no bloqueado
        v, _, alert = self.go(0.14)
        self.assertEqual((v, alert), (0.0, 'obstacle_stop'))
        for d in (0.16, 0.18, 0.20):                         # dentro de la banda: sigue bloqueado
            v, _, alert = self.go(d)
            self.assertEqual((v, alert), (0.0, 'obstacle_stop'), d)
        v, _, alert = self.go(0.21)                          # sobre 0.20: se libera (con rampa)
        self.assertNotEqual(alert, 'obstacle_stop')

    def test_sin_haber_estado_bloqueado_0_17_no_bloquea(self):
        for _ in range(10):
            self.go(0.40)
        v, _, alert = self.go(0.17)
        self.assertNotEqual(alert, 'obstacle_stop')

    def test_una_lectura_oscilando_en_0_15_a_0_19_no_se_destraba(self):
        self.go(0.14)
        out = [self.go(d)[0] for d in (0.16, 0.19, 0.15, 0.18, 0.16, 0.19)]
        self.assertTrue(all(v == 0.0 for v in out))

    def test_sin_histeresis_configurada_vuelve_al_comportamiento_anterior(self):
        p = SafetyParams(sonar_stop_hysteresis=0.0)
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'center': 0.14}, p)
        v, _, alert = gate_step(lim, 0.05, 0.3, {'center': 0.16}, p)
        self.assertNotEqual(alert, 'obstacle_stop')

    def test_cada_sonar_tiene_su_histeresis(self):
        p = SafetyParams(sonar_thresholds={'left': (0.20, 0.08)})
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'left': 0.07, 'center': 1.5}, p)
        _, _, alert = gate_step(lim, 0.05, 0.3, {'left': 0.12, 'center': 1.5}, p)       # 0.08 + 0.05 = 0.13
        self.assertEqual(alert, 'obstacle_stop')
        _, _, alert = gate_step(lim, 0.10, 0.3, {'left': 0.14, 'center': 1.5}, p)
        self.assertNotEqual(alert, 'obstacle_stop')

    def test_sonar_ignorado_olvida_su_bloqueo(self):
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'left': 0.05, 'center': 1.5})
        gate_step(lim, 0.05, 0.3, {'center': 1.5})                      # el lateral deja de contar (perimetro)
        _, _, alert = gate_step(lim, 0.10, 0.3, {'left': 0.17, 'center': 1.5})
        self.assertNotEqual(alert, 'obstacle_stop')

    def test_lectura_nan_conserva_el_bloqueo(self):
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'center': 0.10})
        v, _, alert = gate_step(lim, 0.05, 0.3, {'center': float('nan')})
        self.assertEqual((v, alert), (0.0, 'obstacle_stop'))


class Ramp(unittest.TestCase):
    def test_baja_de_inmediato(self):
        lim = SonarLimiter()
        v0, _, _ = gate_step(lim, 0.0, 0.3, {'center': 1.5})
        v1, _, _ = gate_step(lim, DT, 0.3, {'center': 0.20})
        self.assertAlmostEqual(v0, 0.3)
        self.assertAlmostEqual(v1, 0.3 * (0.20 - 0.15) / 0.15)

    def test_sube_con_rampa_de_a_lo_sumo_la_aceleracion(self):
        p = SafetyParams(sonar_release_accel=0.5)
        lim = SonarLimiter()
        t = 0.0
        gate_step(lim, t, 0.3, {'center': 0.16}, p)                     # casi parado
        t += DT
        prev_allowed = gate_step(lim, t, 0.5, {'center': 0.16}, p)[0]
        speeds = []
        for _ in range(40):                                              # el obstaculo desaparece
            t += DT
            v = gate_step(lim, t, 0.5, {'center': 1.5}, p)[0]
            speeds.append(v)
            self.assertLessEqual(v - prev_allowed, 0.5 * DT + 1e-9, 'sube como maximo accel * dt')
            prev_allowed = v
        self.assertAlmostEqual(speeds[-1], 0.5)                          # termina libre
        self.assertLess(speeds[4], 0.5 * 0.5)                            # tras 0.25 s sigue lejos del maximo
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(speeds, speeds[1:])), 'sube sin oscilar')

    def test_sin_rampa_configurada_sube_de_golpe(self):
        p = SafetyParams(sonar_release_accel=0.0)
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'center': 0.2}, p)
        self.assertAlmostEqual(gate_step(lim, DT, 0.3, {'center': 1.5}, p)[0], 0.3)

    def test_nunca_amplifica_ni_invierte(self):
        rng = random.Random(7)
        lim = SonarLimiter()
        t = 0.0
        for _ in range(4000):
            t += DT
            v_in = rng.choice([-0.5, -0.2, 0.0, 0.1, 0.3, 0.5, 0.8])
            w_in = rng.uniform(-2, 2)
            ranges = {s: rng.choice([0.05, 0.14, 0.16, 0.2, 0.25, 0.4, 1.5, float('nan')]) for s in ('left', 'center', 'right')}
            st = SafetyState(sonar_ranges=ranges, front_pressed=rng.random() < 0.05, back_pressed=rng.random() < 0.05,
                             stale=rng.random() < 0.03)
            v, w, _ = apply_safety(v_in, w_in, st, SafetyParams(), limiter=lim, now=t)
            self.assertLessEqual(abs(v), abs(v_in) + 1e-12)
            self.assertGreaterEqual(v * v_in, 0.0, 'nunca invierte el signo')
            self.assertLessEqual(abs(w), abs(w_in) + 1e-12)
            if v_in < 0 and not st.back_pressed:
                self.assertEqual(v, max(-0.5, v_in))            # el retroceso no depende de los sonares

    def test_bumper_o_dato_viejo_ponen_el_factor_en_cero_y_se_libera_con_rampa(self):
        lim = SonarLimiter()
        gate_step(lim, 0.0, 0.3, {'center': 1.5})
        v, _, alert = gate_step(lim, DT, 0.3, {'center': 1.5}, front_pressed=True)
        self.assertEqual((v, alert), (0.0, 'bumper_front'))
        v, _, _ = gate_step(lim, 2 * DT, 0.3, {'center': 1.5})
        self.assertLess(v, 0.3)                                          # no de golpe


# ── Lecturas ruidosas simuladas: del sonar al gate, con y sin filtro ──

def noisy(true_d, rng, sigma=0.03, p_low=0.06, p_drop=0.08):
    """Lectura de sonar con ruido, picos hacia abajo y perdidas de eco (max_range)."""
    r = rng.random()
    if r < p_drop:
        return 1.5
    if r < p_drop + p_low:
        return max(0.03, true_d - rng.uniform(0.10, 0.30))
    return max(0.03, true_d + rng.gauss(0.0, sigma))


def hold_scene(true_d, seconds, use_filter, seed, v_cmd=0.2):
    """Obstaculo fijo a true_d; el avance pedido es constante. Devuelve las velocidades de salida del gate."""
    rng = random.Random(seed)
    filt = SonarFilter()
    lim = SonarLimiter() if use_filter else None
    p = SafetyParams()
    reading, out, t, next_sonar = 1.5, [], 0.0, 0.0
    for _ in range(int(seconds / DT)):
        t += DT
        if t >= next_sonar:
            next_sonar += SONAR_DT
            raw = noisy(true_d, rng)
            if use_filter:
                f = filt.update(raw)
                reading = reading if f is None else f
            else:
                reading = raw
        v, _, _ = apply_safety(v_cmd, 0.0, SafetyState(sonar_ranges={'center': reading}), p, limiter=lim, now=t)
        out.append(v)
    return out


def reversals(series, eps=0.01):
    """Cambios de sentido (sube/baja) de mas de eps: mide cuanto 'tiembla' la velocidad."""
    n, last, direction = 0, series[0], 0
    for x in series[1:]:
        if abs(x - last) < eps:
            continue
        d = 1 if x > last else -1
        if direction and d != direction:
            n += 1
        direction, last = d, x
    return n


def spread(series, lo=5, hi=95):
    """Ancho de la banda entre dos percentiles: ignora los eventos raros (p. ej. tres perdidas de eco seguidas)."""
    s_ = sorted(series)
    return s_[int(len(s_) * hi / 100) - 1] - s_[int(len(s_) * lo / 100)]


class NoisyReadings(unittest.TestCase):
    SEEDS = range(8)

    def stats(self, true_d, use_filter):
        out = [hold_scene(true_d, 60.0, use_filter, seed)[200:] for seed in self.SEEDS]
        mean = lambda xs: sum(xs) / len(xs)                      # noqa: E731
        return (mean([statistics.pstdev(o) for o in out]), mean([reversals(o, 0.02) for o in out]),
                mean([spread(o) for o in out]))

    def test_la_velocidad_de_salida_no_oscila_con_lecturas_ruidosas(self):
        for true_d in (0.22, 0.28, 0.34):              # cerca de los umbrales de 0.30 y 0.15 + 0.05
            std_raw, rev_raw, band_raw = self.stats(true_d, False)
            std_f, rev_f, band_f = self.stats(true_d, True)
            self.assertLess(std_f, 0.6 * std_raw, (true_d, std_raw, std_f))
            self.assertLess(rev_f, 0.4 * rev_raw, (true_d, rev_raw, rev_f))
            self.assertLess(band_f, 0.7 * band_raw, (true_d, band_raw, band_f))

    def test_con_filtro_la_velocidad_queda_cerca_de_la_esperada(self):
        for true_d in (0.22, 0.28):
            expected = 0.2 * (true_d - 0.15) / 0.15
            for seed in self.SEEDS:
                v = hold_scene(true_d, 60.0, True, seed)[200:]
                self.assertLess(abs(statistics.median(v) - expected), 0.03, (true_d, seed))
                self.assertLess(spread(v), 0.125, (true_d, seed))         # banda 5-95 % (sin filtro llega a 0.2)

    def test_en_el_umbral_de_parada_el_filtro_mas_la_histeresis_no_hacen_tartamudear(self):
        # obstaculo fijo a 0.17 m (entre 0.15 y 0.20): el bloqueo no entra y sale una y otra vez
        raw = hold_scene(0.17, 60.0, False, 2)[100:]
        filt = hold_scene(0.17, 60.0, True, 2)[100:]
        self.assertLess(reversals(filt, 0.01), reversals(raw, 0.01) // 3)

    def test_sin_filtro_el_mismo_ruido_si_hace_oscilar(self):
        # comprobacion del propio test: el ruido simulado es lo bastante malo como para hacer temblar al robot sin filtro
        self.assertGreater(self.stats(0.28, False)[1], 40)


def approach(v_cmd, start, seconds, use_filter, seed, sigma=0.02):
    """El robot avanza hacia una pared (o hacia un obstaculo que aparece de golpe). Devuelve (distancia_min, v_final, trayectoria)."""
    rng = random.Random(seed)
    filt, lim, p = SonarFilter(), (SonarLimiter() if use_filter else None), SafetyParams()
    d, t, next_sonar, reading, traj = start, 0.0, 0.0, 1.5, []
    for _ in range(int(seconds / DT)):
        t += DT
        true_d = d
        if t >= next_sonar:
            next_sonar += SONAR_DT
            raw = noisy(true_d, rng, sigma=sigma, p_low=0.05, p_drop=0.05) if true_d > 0.4 else max(0.03, true_d + rng.gauss(0, sigma))
            if use_filter:
                f = filt.update(raw)
                reading = reading if f is None else f
            else:
                reading = raw
        v, _, _ = apply_safety(v_cmd, 0.0, SafetyState(sonar_ranges={'center': reading}), p, limiter=lim, now=t)
        d = max(0.0, d - v * DT)
        traj.append((t, d, v))
    return min(x[1] for x in traj), traj[-1][2], traj


class RealObstacle(unittest.TestCase):
    # distancia minima exigida al frenar, segun la velocidad pedida (el retraso del filtro pesa mas a 0.5 m/s)
    MIN_GAP = {0.15: 0.10, 0.3: 0.08, 0.5: 0.03}

    def test_un_obstaculo_real_frena_a_tiempo_con_ruido(self):
        for v_cmd, min_gap in self.MIN_GAP.items():
            for seed in range(8):
                dmin, v_end, traj = approach(v_cmd, 2.0, 40.0, True, seed)
                self.assertGreater(dmin, min_gap, (v_cmd, seed, dmin))   # no llega a tocar (el bumper es la ultima barrera)
                self.assertLess(v_end, 0.02)

    def test_con_filtro_frena_casi_igual_que_sin_filtro(self):
        for seed in range(8):
            d_raw = approach(0.3, 2.0, 40.0, False, seed)[0]
            d_flt = approach(0.3, 2.0, 40.0, True, seed)[0]
            self.assertLess(d_raw - d_flt, 0.05, (seed, d_raw, d_flt))        # el filtro no puede quitar mas de 5 cm de margen

    def test_obstaculo_que_aparece_de_golpe_se_detecta_a_la_segunda_lectura(self):
        # camino libre (max_range) y de repente una persona a 0.5 m: dos lecturas bajas seguidas la confirman
        rng = random.Random(0)
        filt, lim, p = SonarFilter(), SonarLimiter(), SafetyParams()
        for _ in range(4):
            filt.update(1.5)
        reading = 1.5
        d, t, next_sonar, v = 0.5, 0.0, 0.0, 0.3
        detected_at = None
        for _ in range(int(20.0 / DT)):
            t += DT
            if t >= next_sonar:
                next_sonar += SONAR_DT
                f = filt.update(max(0.03, d + rng.gauss(0, 0.01)))
                reading = reading if f is None else f
                if detected_at is None and reading < 1.0:
                    detected_at = t
            v, _, _ = apply_safety(0.3, 0.0, SafetyState(sonar_ranges={'center': reading}), p, limiter=lim, now=t)
            d = max(0.0, d - v * DT)
        self.assertIsNotNone(detected_at)
        self.assertLessEqual(detected_at, 2 * SONAR_DT + DT + 1e-9)       # como mucho dos lecturas
        self.assertGreater(d, 0.08)
        self.assertLess(v, 0.02)

    def test_pico_falso_aislado_en_camino_libre_no_frena(self):
        filt, lim, p = SonarFilter(), SonarLimiter(), SafetyParams()
        for _ in range(4):
            filt.update(1.5)
        reading, t, speeds = 1.5, 0.0, []
        seq = [1.5, 1.5, 0.10, 1.5, 1.5, 1.5, 0.20, 1.5, 1.5]            # picos aislados hacia abajo
        for r in seq:
            f = filt.update(r)
            reading = reading if f is None else f
            for _ in range(int(SONAR_DT / DT)):
                t += DT
                speeds.append(apply_safety(0.3, 0.0, SafetyState(sonar_ranges={'center': reading}), p, limiter=lim, now=t)[0])
        self.assertTrue(all(abs(v - 0.3) < 1e-9 for v in speeds), 'los picos aislados no hacen frenar')


if __name__ == '__main__':
    unittest.main()
