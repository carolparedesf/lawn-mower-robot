"""robot/position desde la pose de la simulacion (solo pruebas; el robot real usa fused_position.py: toLL de navsat).

Con la opcion activa, bridge.py toma la TF map -> base_footprint y la convierte a lat/lng con el MISMO datum y la misma
escala (meters_per_degree) que geofence_node en modo datum, asi el robot se ve dentro del cerco que dibujo la app.
Este modulo solo lo importa bridge.py cuando la opcion esta activa: en el robot real no se usa.
Requiere el paquete lawnbot_nav (geofence_logic).

  python3 bridge.py --sim-position --datum-lat -25.3 --datum-lng -57.6
  LAWNBOT_SIM_POSITION=1 LAWNBOT_DATUM_LAT=-25.3 LAWNBOT_DATUM_LNG=-57.6 python3 bridge.py
"""
import argparse

from fused_position import heading_deg
from lawnbot_nav.geofence_logic import lat_lon_to_local_xy, meters_per_degree  # noqa: F401  (lat_lon_to_local_xy: para tests)

TRUE_VALUES = ('1', 'true', 'yes', 'on')
ENV_ENABLE = 'LAWNBOT_SIM_POSITION'
ENV_LAT = 'LAWNBOT_DATUM_LAT'
ENV_LNG = 'LAWNBOT_DATUM_LNG'
SIM_PERIOD_S = 1.0          # robot/position a ~1 Hz, como con gpsd
SIM_FRAME = 'map'
SIM_BASE_FRAME = 'base_footprint'
SIM_POSE_TIMEOUT_S = 2.0


def requested(argv, env):
    """True si se pidio la posicion simulada por argumento o variable de entorno (apagada por defecto)."""
    return '--sim-position' in argv or env.get(ENV_ENABLE, '').strip().lower() in TRUE_VALUES


def parse_options(argv, env):
    """(datum_lat, datum_lng) si la opcion esta activa, None si no. ValueError si falta o es invalido el datum."""
    if not requested(argv, env):
        return None
    ap = argparse.ArgumentParser(prog='bridge.py', add_help=False)
    ap.add_argument('--sim-position', action='store_true')
    ap.add_argument('--datum-lat', type=float, default=None)
    ap.add_argument('--datum-lng', type=float, default=None)
    args, _ = ap.parse_known_args(argv)
    try:
        lat = args.datum_lat if args.datum_lat is not None else float(env[ENV_LAT])
        lng = args.datum_lng if args.datum_lng is not None else float(env[ENV_LNG])
    except (KeyError, ValueError):
        raise ValueError(f'la posicion simulada necesita el datum del geofence_node: --datum-lat y --datum-lng '
                         f'(o {ENV_LAT} y {ENV_LNG}); lo imprime tools/datum_from_fence.py')
    if (lat, lng) == (0.0, 0.0) or not (-90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0):
        raise ValueError(f'datum invalido ({lat}, {lng}): geofence_node en modo datum lo rechaza igual')
    return lat, lng


def xy_to_lat_lon(x, y, datum_lat, datum_lng):
    """Inversa exacta de geofence_logic.lat_lon_to_local_xy (x = este, y = norte, escala de datum_lat)."""
    m_per_deg_lng, m_per_deg_lat = meters_per_degree(datum_lat)
    return datum_lat + y / m_per_deg_lat, datum_lng + x / m_per_deg_lng




def position_payload(x, y, yaw, datum_lat, datum_lng):
    """Mismo JSON que publica el hilo de gpsd: {"lat", "lng", "hdg"}."""
    lat, lng = xy_to_lat_lon(x, y, datum_lat, datum_lng)
    return {'lat': lat, 'lng': lng, 'hdg': heading_deg(yaw)}
