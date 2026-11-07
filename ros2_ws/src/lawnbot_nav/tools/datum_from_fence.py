#!/usr/bin/env python3
"""Calcula datum_lat/datum_lng para que el cerco guardado por la app caiga en el centro del mundo simulado.

Lee ~/.ros/lawnbot_geofence.json (lat/lon), pone el centro del cerco en el centro del mundo elegido
(config/sim_room.yaml o sim_field.yaml, segun --world), comprueba que el arranque del robot quede dentro del cerco
con un margen minimo y que el cerco entre en el mundo, e imprime los comandos de sim_world y nav.launch.py.
Sin ROS: solo usa la logica de lawnbot_nav.geofence_logic.

  python3 tools/datum_from_fence.py --world field
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from lawnbot_nav.geofence_logic import (drop_closing_vertex, lat_lon_to_local_xy,  # noqa: E402
                                        load_geofence, meters_per_degree, polygon_centroid,
                                        signed_distance, validate_polygon)

WORLDS = ('room', 'field')
DEFAULT_FENCE = '~/.ros/lawnbot_geofence.json'
DEFAULT_MARGIN = 0.4                                  # m entre el arranque del robot y el borde del cerco


class DatumError(Exception):
    pass


def load_world(name, config_dir=None):
    """(centro, arranque, caja) del mundo: centro = centro de la caja de las paredes, arranque = x, y del YAML."""
    if name not in WORLDS:
        raise DatumError(f"world '{name}' invalido: use {' o '.join(WORLDS)}")
    config_dir = config_dir or os.path.join(HERE, '..', 'config')
    path = os.path.join(config_dir, f'sim_{name}.yaml')
    with open(path) as f:
        params = yaml.safe_load(f)['fake_robot']['ros__parameters']
    walls = [float(v) for v in params['walls']]
    xs = walls[0::2]
    ys = walls[1::2]
    box = (min(xs), min(ys), max(xs), max(ys))
    center = ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)
    return center, (float(params['x']), float(params['y'])), box


def local_to_lat_lon(x, y, lat0, lng0):
    """Inversa de lat_lon_to_local_xy (la escala es la de lat0)."""
    m_lng, m_lat = meters_per_degree(lat0)
    return lat0 + y / m_lat, lng0 + x / m_lng


def datum_for_center(points, center):
    """Datum (lat, lng) con el que el centro del cerco (centroide del poligono) cae en `center` = (x, y).

    La escala en metros depende de la latitud del datum, asi que se corrige iterando hasta que el centroide
    calculado con la misma conversion que usa geofence_node coincida con `center`.
    """
    lat0 = sum(p[0] for p in points) / len(points)
    lng0 = sum(p[1] for p in points) / len(points)
    for _ in range(10):
        cx, cy = polygon_centroid([lat_lon_to_local_xy(la, ln, lat0, lng0) for la, ln in points])
        dlat, dlng = local_to_lat_lon(cx - center[0], cy - center[1], lat0, lng0)
        lat0, lng0 = dlat, dlng                      # mover el datum por el error del centroide
        if abs(cx - center[0]) < 1e-6 and abs(cy - center[1]) < 1e-6:
            break
    return lat0, lng0


def compute(points, world, margin=DEFAULT_MARGIN, config_dir=None):
    """Devuelve un dict con el datum y los controles. DatumError si el cerco no sirve para este mundo."""
    center, start, box = load_world(world, config_dir)
    points = drop_closing_vertex(points, 1e-9)
    lat0, lng0 = datum_for_center(points, center)
    xy = [lat_lon_to_local_xy(lat, lng, lat0, lng0) for lat, lng in points]
    poly, reason = validate_polygon(xy)
    if poly is None:
        raise DatumError(f'cerco invalido: {reason}')
    fence_center = polygon_centroid(poly)
    outside = [i + 1 for i, (x, y) in enumerate(poly) if not (box[0] < x < box[2] and box[1] < y < box[3])]
    if outside:
        raise DatumError(f'el cerco no entra en el mundo {world} ({box[0]:g}..{box[2]:g} x {box[1]:g}..{box[3]:g} m): '
                         f'vertices {outside} quedan fuera de las paredes')
    start_margin = signed_distance(start, poly)
    if start_margin < margin:
        raise DatumError(f'el arranque del robot ({start[0]:g}, {start[1]:g}) queda a {start_margin:.2f} m del borde '
                         f'del cerco (minimo {margin:g} m; negativo = fuera)')
    return {'world': world, 'datum_lat': lat0, 'datum_lng': lng0, 'center': center, 'start': start,
            'start_margin': start_margin, 'fence_center': fence_center, 'polygon': poly}


def commands(result):
    return [f"ros2 launch lawnbot_nav sim_world.launch.py world:={result['world']}",
            'ros2 launch lawnbot_nav nav.launch.py geofence_conversion_mode:=datum '
            f"datum_lat:={result['datum_lat']:.9f} datum_lng:={result['datum_lng']:.9f}"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', choices=WORLDS, default='room', help='mundo simulado (por defecto room)')
    ap.add_argument('--fence', default=DEFAULT_FENCE, help=f'JSON del cerco en lat/lon (por defecto {DEFAULT_FENCE})')
    ap.add_argument('--margin', type=float, default=DEFAULT_MARGIN,
                    help=f'margen minimo [m] del arranque al borde (por defecto {DEFAULT_MARGIN})')
    args = ap.parse_args(argv)
    points = load_geofence(args.fence)
    if points is None:
        print(f'error: no se pudo leer un cerco valido de {os.path.expanduser(args.fence)}', file=sys.stderr)
        return 1
    try:
        r = compute(points, args.world, args.margin)
    except DatumError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    print(f"mundo {r['world']}: centro ({r['center'][0]:g}, {r['center'][1]:g}), arranque ({r['start'][0]:g}, {r['start'][1]:g}), "
          f"{len(r['polygon'])} vertices")
    print(f"centro del cerco en el mundo: ({r['fence_center'][0]:.2f}, {r['fence_center'][1]:.2f}); "
          f"el arranque queda a {r['start_margin']:.2f} m del borde (minimo {args.margin:g} m)")
    print()
    for line in commands(r):
        print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
