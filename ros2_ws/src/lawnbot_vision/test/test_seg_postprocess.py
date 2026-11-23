"""Post proceso de YOLOv8-seg con salidas sinteticas (sin modelo). Contra ultralytics se comparo aparte con el
modelo real: mismas clases, puntajes, cajas y mascaras (IoU 1,0)."""
import unittest

try:
    import cv2  # noqa: F401
    import numpy as np
    from lawnbot_vision import seg_postprocess as sp
    HAVE_CV = True
except ImportError:
    HAVE_CV = False

NC, NM, S = 7, 32, 320
GRASS, TREE = 4, 0


def outputs(dets):
    """dets: (clase, puntaje, cx, cy, w, h, valor_de_mascara). Mascara constante: un prototipo lleno de 1."""
    n = 50
    out0 = np.zeros((4 + NC + NM, n), dtype=np.float32)
    for j, (cls, score, cx, cy, w, h, m) in enumerate(dets):
        out0[:4, j] = (cx, cy, w, h)
        out0[4 + cls, j] = score
        out0[4 + NC, j] = m                       # coeficiente del prototipo 0
    protos = np.zeros((NM, S // 4, S // 4), dtype=np.float32)
    protos[0] = 1.0
    return out0, protos


@unittest.skipUnless(HAVE_CV, 'requiere numpy y OpenCV')
class Letterbox(unittest.TestCase):
    def test_640x480_queda_centrado_con_relleno_arriba_y_abajo(self):
        blob, lb = sp.letterbox(np.zeros((480, 640, 3), np.uint8), S)
        self.assertEqual(blob.shape, (3, S, S))
        self.assertEqual((lb.gain, lb.left, lb.top, lb.new_w, lb.new_h), (0.5, 0, 40, 320, 240))
        self.assertAlmostEqual(float(blob[0, 0, 0]), 114 / 255, places=5)       # relleno gris
        self.assertEqual(float(blob[0, 100, 100]), 0.0)                         # imagen

    def test_rgb(self):
        img = np.zeros((S, S, 3), np.uint8)
        img[..., 2] = 255                                                       # rojo en BGR
        blob, _ = sp.letterbox(img, S)
        self.assertEqual(float(blob[0].mean()), 1.0)
        self.assertEqual(float(blob[2].mean()), 0.0)


@unittest.skipUnless(HAVE_CV, 'requiere numpy y OpenCV')
class Decode(unittest.TestCase):
    def test_filtra_por_confianza(self):
        out0, protos = outputs([(TREE, 0.9, 100, 100, 40, 40, 5.0), (TREE, 0.2, 200, 200, 40, 40, 5.0)])
        dets = sp.decode(out0, protos, NC, conf=0.4)
        self.assertEqual([(d.cls, round(d.score, 2)) for d in dets], [(TREE, 0.9)])

    def test_nms_por_clase(self):
        out0, protos = outputs([(TREE, 0.9, 100, 100, 40, 40, 5.0), (TREE, 0.8, 102, 100, 40, 40, 5.0),
                                (GRASS, 0.7, 100, 100, 40, 40, 5.0)])
        dets = sp.decode(out0, protos, NC, conf=0.4)
        self.assertEqual(sorted(d.cls for d in dets), [TREE, GRASS])          # el arbol repetido se va

    def test_mascara_recortada_a_la_caja(self):
        out0, protos = outputs([(TREE, 0.9, 100, 150, 40, 60, 5.0)])
        d = sp.decode(out0, protos, NC, conf=0.4)[0]
        ys, xs = np.nonzero(d.mask)
        self.assertEqual((xs.min(), xs.max(), ys.min(), ys.max()), (80, 119, 120, 179))

    def test_mascara_negativa_vacia(self):
        out0, protos = outputs([(TREE, 0.9, 100, 150, 40, 60, -5.0)])
        self.assertFalse(sp.decode(out0, protos, NC, conf=0.4)[0].mask.any())

    def test_forma_inesperada(self):
        out0, protos = outputs([])
        with self.assertRaises(ValueError):
            sp.decode(out0, protos, NC + 1, conf=0.4)


@unittest.skipUnless(HAVE_CV, 'requiere numpy y OpenCV')
class GrassAndBase(unittest.TestCase):
    def setUp(self):
        _, self.lb = sp.letterbox(np.zeros((480, 640, 3), np.uint8), S)        # imagen en filas 40..279

    def test_fraccion_de_pasto_es_la_union_sin_relleno(self):
        out0, protos = outputs([(GRASS, 0.9, 160, 220, 320, 120, 5.0),        # mitad de abajo (filas 160..279)
                                (GRASS, 0.8, 160, 300, 320, 40, 5.0),          # solo relleno: no cuenta
                                (TREE, 0.9, 160, 100, 320, 40, 5.0)])
        dets = sp.decode(out0, protos, NC, conf=0.4, iou=0.99)
        self.assertAlmostEqual(sp.grass_fraction(dets, GRASS, self.lb), 0.5, places=2)

    def test_sin_pasto(self):
        self.assertEqual(sp.grass_fraction([], GRASS, self.lb), 0.0)

    def test_punto_base_en_pixeles_de_la_imagen_original(self):
        out0, protos = outputs([(TREE, 0.9, 100, 150, 40, 60, 5.0)])         # entrada: x 80..119, y 120..179
        d = sp.decode(out0, protos, NC, conf=0.4)[0]
        x, y = sp.base_point(d.mask, self.lb)
        self.assertAlmostEqual(y, (179 - 40) / 0.5)                             # fila mas baja, sin el relleno
        self.assertAlmostEqual(x, (80 + 119) / 2 / 0.5)

    def test_mascara_vacia_sin_punto(self):
        self.assertIsNone(sp.base_point(np.zeros((S, S), bool), self.lb))


if __name__ == '__main__':
    unittest.main()
