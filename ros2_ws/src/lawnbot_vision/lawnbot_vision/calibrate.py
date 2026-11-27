#!/usr/bin/env python3
"""Calibracion de distancia de la camara para vision_node (reemplazo del viejo vision/pi_scripts/calibrar.py).

Usa EXACTAMENTE lo mismo que vision_node: el detector NCNN (detector.NcnnSegDetector), el punto de apoyo
(seg_postprocess.base_point), la camara (detector.Camera, por su nombre fijo y con V4L2) y los parametros de
vision.yaml (modelo, imgsz, conf, resolucion de la camara, clases de obstaculo). Corre con el python del sistema
(el de ROS), sin ultralytics.

Que mide: para cada distancia, la fila (pixel y) del punto de apoyo del objeto. vision_node interpola entre esos
pares (calib_y, calib_d) para saber a cuantos metros esta un obstaculo.

Las distancias se miden SOBRE EL PISO, desde el punto que esta justo debajo del lente de la camara hasta donde el
objeto toca el piso, con el objeto CENTRADO en la imagen (al frente de la camara). vision_node toma esa distancia
como medida desde la camara y despues le suma la posicion de la camara en el robot (cam_x, cam_y, cam_yaw_deg).

Uso (en la Pi, con vision_node PARADO: la camara no se puede abrir desde dos programas):
    ros2 run lawnbot_vision calibrar
    ros2 run lawnbot_vision calibrar --distancias 0.4 0.7 1.0 1.5 2.0 --cam-x 0.20 --cam-y 0.0
"""
import argparse
import os
import statistics
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from lawnbot_vision.vision_logic import validate_calibration

DEFAULT_DISTANCES = (0.5, 1.0, 1.5, 2.0)
OFF_CENTER = 0.10            # fraccion del ancho: mas lejos del centro que esto, se avisa


# ── Logica pura (se prueba sin camara ni modelo) ────────────────────────────

def load_params(path: str) -> Dict:
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    try:
        return dict(data['vision_node']['ros__parameters'])
    except (KeyError, TypeError):
        raise ValueError(f'{path}: falta vision_node/ros__parameters')


def default_config() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('lawnbot_vision'), 'config', 'vision.yaml')
    except Exception:                       # sin instalar: el del arbol de fuentes
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'vision.yaml')


def pick_target(candidates: Sequence[Tuple[str, float, float, int]], classes: Sequence[str]):
    """El objeto de calibracion: la mascara mas grande entre las clases pedidas (como el script viejo, que tomaba
    la mas grande). candidates: (clase, x_base, y_base, area_px). None si no hay ninguno."""
    ok = [c for c in candidates if c[0] in classes]
    return max(ok, key=lambda c: c[3]) if ok else None


def summarize(ys: Sequence[float]) -> Tuple[float, float]:
    """(mediana, dispersion = max - min) de las filas medidas en una distancia."""
    return float(statistics.median(ys)), float(max(ys) - min(ys))


def build_result(points: Sequence[Tuple[float, float]]) -> Tuple[List[float], List[float], Optional[str]]:
    """points: (y_base, distancia). Ordena de lejos a cerca (y creciente) y valida con la MISMA funcion que
    vision_node. Devuelve (calib_y, calib_d, error o None)."""
    pts = sorted(points)
    calib_y = [float(round(y)) for y, _ in pts]
    calib_d = [float(d) for _, d in pts]
    return calib_y, calib_d, validate_calibration(calib_y, calib_d)


def yaml_block(calib_y, calib_d, width, height, cam=None) -> str:
    """Lo que hay que pegar en vision.yaml, debajo de ros__parameters (misma indentacion)."""
    ys = '[' + ', '.join(f'{y:.1f}' for y in calib_y) + ']'
    ds = '[' + ', '.join(f'{float(d):.2f}' for d in calib_d) + ']'
    lines = [
        f'    # Calibracion de distancia ({width}x{height}: vale solo con camera_width/camera_height iguales)',
        '    calibrated: true',
        f'    calib_y: {ys}',
        f'    calib_d: {ds}',
    ]
    if cam:
        lines += [f'    cam_x: {cam[0]:.3f}', f'    cam_y: {cam[1]:.3f}', f'    cam_yaw_deg: {cam[2]:.1f}']
    else:
        lines += ['    # cam_x, cam_y, cam_yaw_deg: posicion de la camara en base_footprint (ver --cam-x/--cam-y)']
    return '\n'.join(lines)


# ── Camara y modelo ─────────────────────────────────────────────────────────

def measure(detector, camera, classes, samples: int, flush: int) -> Tuple[List[float], List[float], List[str]]:
    from lawnbot_vision import seg_postprocess as sp
    for _ in range(flush):                  # cuadros viejos de la cola / exposicion automatica que se acomoda
        camera.read()
    ys, xs, names = [], [], []
    for _ in range(samples):
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError('la camara no entrego imagen')
        lb, dets = detector.segment(frame)
        cands = []
        for d in dets:
            bp = sp.base_point(d.mask, lb)
            if bp is not None:
                cands.append((detector.names[d.cls], bp[0], bp[1], int(d.mask.sum())))
        t = pick_target(cands, classes)
        if t is not None:
            names.append(t[0])
            xs.append(t[1])
            ys.append(t[2])
    return ys, xs, names


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--config', default=default_config(), help='vision.yaml (por defecto el instalado)')
    ap.add_argument('--source', help='camara o video (por defecto el source de vision.yaml)')
    ap.add_argument('--model', help='carpeta del modelo (por defecto la de vision.yaml)')
    ap.add_argument('--distancias', type=float, nargs='+', default=list(DEFAULT_DISTANCES),
                    help='metros, medidos en el piso desde debajo del lente de la camara')
    ap.add_argument('--clases', nargs='+', help='clases que cuentan como el objeto (por defecto obstacle_classes)')
    ap.add_argument('--muestras', type=int, default=8)
    ap.add_argument('--descartar', type=int, default=10, help='cuadros a descartar antes de medir')
    ap.add_argument('--cam-x', type=float, help='m, camara delante (+) del origen de base_footprint')
    ap.add_argument('--cam-y', type=float, default=0.0, help='m, camara a la izquierda (+)')
    ap.add_argument('--cam-yaw', type=float, default=0.0, help='grados, giro de la camara (+ a la izquierda)')
    ap.add_argument('--sin-pausa', action='store_true', help='no esperar Enter (pruebas con un video)')
    args = ap.parse_args(argv)

    os.environ.setdefault('OMP_NUM_THREADS', '1')
    p = load_params(args.config)
    source = args.source or str(p.get('source', '0'))
    model = args.model or p.get('model', '')
    classes = args.clases or list(p.get('obstacle_classes', ['arbol', 'planta', 'barrera', 'otro']))
    print(f'config: {args.config}')

    import cv2
    cv2.setNumThreads(1)
    from lawnbot_vision.detector import Camera, NcnnSegDetector
    detector = NcnnSegDetector(model, int(p.get('imgsz', 320)), float(p.get('conf', 0.4)), float(p.get('iou', 0.7)),
                               int(p.get('threads', 1)), str(p.get('grass_class', 'pasto')))
    print(f'modelo: {model} | clases del objeto: {classes}')
    try:
        camera = Camera(source, int(p.get('camera_width', 640)), int(p.get('camera_height', 480)),
                        float(p.get('camera_fps', 15.0)), str(p.get('camera_fourcc', 'MJPG')))
        ok, frame = camera.read()
        if not ok:
            raise RuntimeError('abrio pero no entrega imagen')
    except Exception as exc:
        print(f'\nERROR: no se pudo usar la camara {source}: {exc}\n'
              '  - ¿Esta corriendo vision_node (o nav.launch.py con vision:=true)? Pararlo: la camara no se\n'
              '    puede usar desde dos programas a la vez.\n'
              '  - ¿Esta conectada? ls /dev/v4l/by-id/', file=sys.stderr)
        return 1
    width, height = frame.shape[1], frame.shape[0]
    print(f'camara: {source} {camera.describe()}\n')
    print('Medi cada distancia SOBRE EL PISO desde el punto justo debajo del lente de la camara hasta donde el\n'
          'objeto toca el piso. Objeto CENTRADO en la imagen. Usá un objeto que el modelo reconozca como\n'
          f'{"/".join(classes)} (por ejemplo una maceta o una caja), siempre el mismo.\n')

    points = []
    for d in args.distancias:
        while True:
            if not args.sin_pausa:
                r = input(f'Objeto con su BASE sobre la marca de {d} m. Enter = medir, s = saltar: ').strip().lower()
                if r == 's':
                    break
            ys, xs, names = measure(detector, camera, classes, args.muestras, args.descartar)
            if len(ys) < max(3, args.muestras // 2):
                print(f'   detectado en {len(ys)}/{args.muestras} cuadros: muy poco. Acomodá el objeto o la luz.')
                if args.sin_pausa:
                    break
                continue
            y, spread = summarize(ys)
            x = statistics.median(xs)
            off = (x - width / 2) / width
            print(f'   d = {d} m -> y_base = {y:.0f} px (dispersion {spread:.0f} px, {len(ys)}/{args.muestras} '
                  f'cuadros, clase {statistics.mode(names)}, x = {x:.0f} de {width})')
            if abs(off) > OFF_CENTER:
                print(f'   AVISO: el objeto esta {"a la derecha" if off > 0 else "a la izquierda"} del centro '
                      f'({abs(off) * 100:.0f} % del ancho); centralo y repetí para que la distancia sea al frente.')
            if spread > 0.05 * height:
                print('   AVISO: las filas varian mucho entre cuadros; conviene repetir.')
            if not args.sin_pausa and input('   ¿Usar este punto? [S/n]: ').strip().lower() == 'n':
                continue
            points.append((y, d))
            break
    camera.release()

    if len(points) < 2:
        print('\nHacen falta al menos 2 distancias medidas.', file=sys.stderr)
        return 1
    calib_y, calib_d, err = build_result(points)
    if err:
        print(f'\nNO SIRVE: {err}. calib_y={calib_y} calib_d={calib_d}\n'
              'Mas lejos tiene que dar una fila mas chica (mas arriba en la imagen). Repetí las mediciones.',
              file=sys.stderr)
        return 1
    cam = None if args.cam_x is None else (args.cam_x, args.cam_y, args.cam_yaw)
    print('\n' + '=' * 72)
    print(f'Pegar en {args.config.replace(os.path.expanduser("~"), "~")}\n'
          '(o en ros2_ws/src/lawnbot_vision/config/vision.yaml del repo y recompilar), reemplazando esas claves:\n')
    print(yaml_block(calib_y, calib_d, width, height, cam))
    print('=' * 72)
    print(f'Rango calibrado: {calib_d[-1]} a {calib_d[0]} m. Mas lejos que {calib_d[0]} m no se publica el obstaculo;'
          f'\nmas cerca que {calib_d[-1]} m se usa {calib_d[-1]} m.')
    if cam is None:
        print('Falta cam_x/cam_y/cam_yaw_deg: medilos en el robot (ver la ayuda: --cam-x, --cam-y, --cam-yaw).')
    print(f'fov_h_deg sigue en {p.get("fov_h_deg", 62.0)} (supuesto): afecta el angulo, no la distancia.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
