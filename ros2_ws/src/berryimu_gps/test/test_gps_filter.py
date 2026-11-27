"""Filtro de fixes de gpsd (gps_filter.py) y su uso en gps_node (sin gpsd, TPV/SKY a mano)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from berryimu_gps import gps_filter as gf  # noqa: E402

LAT, LON = -25.2637, -57.5759
M_PER_DEG_LAT = gf.EARTH_RADIUS_M * math.pi / 180.0


def tpv(dlat_m=0.0, mode=3, **extra):
    """TPV con un fix a dlat_m metros al norte del punto base."""
    report = {'class': 'TPV', 'mode': mode, 'lat': LAT + dlat_m / M_PER_DEG_LAT, 'lon': LON,
              'epx': 2.0, 'epy': 2.0, 'eph': 3.0, 'epv': 4.0}
    report.update(extra)
    return report


class Covarianza(unittest.TestCase):
    def test_ep_al_95_a_sigma(self):
        sx, sy, vvar, known = gf.sigmas({'epx': 3.0, 'epy': 5.0, 'epv': 6.0}, gf.FilterParams())
        self.assertEqual((sx, sy, known), (1.5, 2.5, True))
        self.assertAlmostEqual(vvar, 9.0)

    def test_sin_epx_epy_usa_eph(self):
        sx, sy, _, known = gf.sigmas({'eph': 4.0}, gf.FilterParams())
        self.assertEqual((sx, sy, known), (2.0, 2.0, False))

    def test_sin_errores_usa_el_sigma_de_respaldo_sin_escalar(self):
        sx, sy, vvar, known = gf.sigmas({}, gf.FilterParams(fallback_sigma_m=10.0))
        self.assertEqual((sx, sy, vvar, known), (10.0, 10.0, 1.0e4, False))

    def test_piso_del_sigma(self):
        sx, sy, _, _ = gf.sigmas({'epx': 0.2, 'epy': 0.2}, gf.FilterParams(min_sigma_m=0.5))
        self.assertEqual((sx, sy), (0.5, 0.5))

    def test_factor_configurable(self):
        sx, _, _, _ = gf.sigmas({'epx': 4.0, 'epy': 4.0}, gf.FilterParams(ep_to_sigma=1.0))
        self.assertEqual(sx, 4.0)


class Calidad(unittest.TestCase):
    def test_sin_fix(self):
        f = gf.FixFilter()
        self.assertEqual(f.check({'mode': 1}, 0.0).reason, gf.REJECT_NO_FIX)
        self.assertEqual(f.check({'mode': 3}, 0.0).reason, gf.REJECT_NO_FIX)    # sin lat/lon

    def test_descarta_2d(self):
        f = gf.FixFilter()
        self.assertEqual(f.check(tpv(mode=2), 0.0).reason, gf.REJECT_2D)
        f2d = gf.FixFilter(gf.FilterParams(require_3d=False))
        self.assertTrue(f2d.check(tpv(mode=2), 0.0).accepted)

    def test_descarta_eph_alto(self):
        f = gf.FixFilter()
        self.assertTrue(f.check(tpv(eph=8.0), 0.0).accepted)                    # en el limite pasa
        self.assertEqual(f.check(tpv(eph=8.1), 1.0).reason, gf.REJECT_EPH)
        free = gf.FixFilter(gf.FilterParams(max_eph_m=0.0))
        self.assertTrue(free.check(tpv(eph=50.0), 0.0).accepted)

    def test_descarta_hdop_alto(self):
        f = gf.FixFilter()
        self.assertTrue(f.check(tpv(), 0.0).accepted)       # sin SKY todavia: no se juzga
        f.on_sky({'class': 'SKY', 'hdop': 2.6})
        self.assertEqual(f.check(tpv(), 1.0).reason, gf.REJECT_HDOP)
        f.on_sky({'class': 'SKY'})                          # SKY sin hdop: queda el anterior
        self.assertEqual(f.check(tpv(), 2.0).reason, gf.REJECT_HDOP)
        f.on_sky({'class': 'SKY', 'hdop': 1.2})
        self.assertTrue(f.check(tpv(), 3.0).accepted)

    def test_un_fix_descartado_no_es_referencia(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        f.check(tpv(100.0, mode=2), 1.0)                                        # 2D: no cuenta
        self.assertTrue(f.check(tpv(0.5), 2.0).accepted)


class Saltos(unittest.TestCase):
    # sigma = 2.0 * 0.5 = 1 m; limite = 0.8 * dt + 3 m

    def test_movimiento_normal_pasa(self):
        f = gf.FixFilter()
        self.assertTrue(f.check(tpv(0.0), 0.0).accepted)
        self.assertTrue(f.check(tpv(3.7), 1.0).accepted)                       # 3.7 < 0.8 + 3

    def test_salto_rechazado(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        d = f.check(tpv(4.0), 1.0)                                              # 4.0 > 3.8
        self.assertEqual(d.reason, gf.REJECT_JUMP)
        self.assertIn('1/5', d.detail)

    def test_el_limite_crece_con_el_tiempo(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        self.assertTrue(f.check(tpv(10.0), 10.0).accepted)                     # 10 < 0.8 * 10 + 3

    def test_el_limite_crece_con_sigma(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        # sigma 2 m: 5 < 0.8 + 6
        self.assertTrue(f.check(tpv(5.0, epx=4.0, epy=4.0), 1.0).accepted)

    def test_acepta_despues_de_5_rechazos_seguidos(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        reasons = [f.check(tpv(20.0), 1.0 + 0.1 * i).reason for i in range(6)]
        self.assertEqual(reasons, [gf.REJECT_JUMP] * 5 + [gf.ACCEPT])
        self.assertTrue(f.check(tpv(20.5), 2.0).accepted)                      # nueva referencia
        self.assertEqual(f.jump_rejects, 0)

    def test_un_fix_bueno_reinicia_la_cuenta(self):
        f = gf.FixFilter()
        f.check(tpv(0.0), 0.0)
        for i in range(4):
            f.check(tpv(20.0), 0.1 * (i + 1))
        self.assertTrue(f.check(tpv(0.5), 0.5).accepted)
        self.assertEqual(f.check(tpv(20.0), 0.6).reason, gf.REJECT_JUMP)
        self.assertEqual(f.jump_rejects, 1)

    def test_sin_rechazo_de_saltos(self):
        f = gf.FixFilter(gf.FilterParams(jump_accept_after=0))
        f.check(tpv(0.0), 0.0)
        self.assertTrue(f.check(tpv(500.0), 0.1).accepted)

    def test_distancia(self):
        d = gf.distance_m(LAT, LON, LAT + 10.0 / M_PER_DEG_LAT, LON)
        self.assertAlmostEqual(d, 10.0, places=6)


try:
    import rclpy
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from berryimu_gps.gps_node import GpsNode
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


@unittest.skipUnless(HAVE_ROS, 'rclpy no disponible')
class Nodo(unittest.TestCase):
    """GpsNode real con gpsd apuntando a un puerto cerrado (el hilo reintenta sin molestar)."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        from rclpy.parameter import Parameter
        self.node = GpsNode(parameter_overrides=[Parameter('gpsd_port', value=1)])
        self.sent = []
        self.node.fix_pub = type('P', (), {'publish': lambda _, m: self.sent.append(m)})()

    def tearDown(self):
        self.node.destroy_node()

    def test_parametros_por_defecto(self):
        p = self.node.filter.p
        self.assertEqual(
            (p.ep_to_sigma, p.max_eph_m, p.max_hdop, p.max_speed, p.jump_accept_after),
            (0.5, 8.0, 2.5, 0.8, 5))

    def test_fix_bueno_con_covarianza_en_sigma(self):
        self.node.publish_fix(tpv(epx=3.0, epy=5.0), now=0.0)
        msg = self.sent[-1]
        self.assertEqual(msg.status.status, NavSatStatus.STATUS_FIX)
        self.assertEqual(list(msg.position_covariance[:5:4]), [2.25, 6.25])
        self.assertEqual(msg.position_covariance_type, NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)

    def test_fix_descartado_no_se_publica(self):
        self.node.publish_fix(tpv(mode=2), now=0.0)
        self.node._handle_line(b'{"class":"SKY","hdop":3.0}')
        self.node.publish_fix(tpv(), now=1.0)
        self.assertEqual(self.sent, [])

    def test_sin_fix_publica_no_fix(self):
        self.node.publish_fix({'class': 'TPV', 'mode': 1}, now=0.0)
        self.assertEqual(self.sent[-1].status.status, NavSatStatus.STATUS_NO_FIX)


if __name__ == '__main__':
    unittest.main()
