"""robot/position desde la localizacion FUSIONADA (EKF + GPS), con el GPS crudo de gpsd como respaldo.

La pose sale de la TF map -> base_footprint (ekf_global) y se pasa a lat/lng con el servicio toLL de navsat_transform:
es la misma transformacion (mismo datum) con la que geofence_node pasa el cerco de lat/lng a `map` (fromLL), asi el
robot se dibuja en la app en el mismo lugar donde lo ve el cerco. El rumbo (hdg) sale del yaw de esa TF: el rumbo
del GPS (track) no sirve a las velocidades del robot (menos de 1 m/s).

Si la TF o toLL no estan disponibles, se publica el fix crudo de gpsd (con su track) y robot/status lo avisa en
"position_source": "fused" (fusionada), "gps_raw" (respaldo) o "none" (sin posicion).

La logica no depende de ROS (se prueba con test_fused_position.py); bridge.py hace las llamadas a ROS y a gpsd.
Va junto a bridge.py (en la Pi: ~/lawn-mower-robot/bridge/).
"""
import math
from dataclasses import dataclass
from typing import Optional

SOURCE_FUSED = 'fused'
SOURCE_GPS_RAW = 'gps_raw'
SOURCE_NONE = 'none'

POSITION_PERIOD_S = 1.0     # robot/position a 1 Hz (como antes con gpsd)
MAP_FRAME = 'map'
BASE_FRAME = 'base_footprint'
TF_TIMEOUT_S = 1.0          # TF map -> base_footprint mas vieja que esto = no disponible (ekf_global va a 10 Hz)
FUSED_STALE_S = 2.5         # sin una respuesta valida de toLL en este tiempo se pasa al GPS crudo
RAW_STALE_S = 3.0           # fix de gpsd mas viejo que esto no se publica
TOLL_SERVICE = 'toLL'
GPSD_RETRY_S = 2.0          # reintento de conexion a gpsd


def heading_deg(yaw):
    """Rumbo como lo da gpsd (grados desde el norte, en sentido horario) a partir del yaw de ROS (desde el este, antihorario)."""
    return (90.0 - math.degrees(yaw)) % 360.0


def yaw_from_quaternion(q):
    """Yaw [rad] de un cuaternion con campos x, y, z, w."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def valid_lat_lng(lat, lng):
    """navsat_transform responde (0, 0) sin error si todavia no tiene su transformacion: eso no es una posicion."""
    try:
        lat, lng = float(lat), float(lng)
    except (TypeError, ValueError):
        return False
    if not (math.isfinite(lat) and math.isfinite(lng)):
        return False
    if lat == 0.0 and lng == 0.0:
        return False
    return -90.0 <= lat <= 90.0 and -180.0 <= lng <= 180.0


@dataclass
class Fix:
    lat: float
    lng: float
    hdg: float
    t: float            # instante (monotonic) en que se obtuvo


class PositionSelector:
    """Elige que publicar en robot/position: la posicion fusionada si es reciente, si no el GPS crudo, si no nada."""

    def __init__(self, fused_stale_s=FUSED_STALE_S, raw_stale_s=RAW_STALE_S):
        self.fused_stale_s = fused_stale_s
        self.raw_stale_s = raw_stale_s
        self.fused: Optional[Fix] = None
        self.raw: Optional[Fix] = None

    def on_fused(self, lat, lng, yaw, now):
        """Respuesta de toLL para una pose de la TF. False si la respuesta no es una posicion valida."""
        if not valid_lat_lng(lat, lng) or not math.isfinite(yaw):
            return False
        self.fused = Fix(float(lat), float(lng), heading_deg(yaw), now)
        return True

    def on_raw(self, lat, lng, track, now):
        """Fix 2D/3D de gpsd. El track puede faltar (None): se publica 0 como antes."""
        if not valid_lat_lng(lat, lng):
            return False
        hdg = float(track) if track is not None and math.isfinite(float(track)) else 0.0
        self.raw = Fix(float(lat), float(lng), hdg % 360.0, now)
        return True

    def source(self, now):
        if self.fused is not None and now - self.fused.t <= self.fused_stale_s:
            return SOURCE_FUSED
        if self.raw is not None and now - self.raw.t <= self.raw_stale_s:
            return SOURCE_GPS_RAW
        return SOURCE_NONE

    def payload(self, now):
        """(fuente, {"lat", "lng", "hdg"} o None). Mismo JSON de siempre para la app."""
        src = self.source(now)
        fix = self.fused if src == SOURCE_FUSED else self.raw if src == SOURCE_GPS_RAW else None
        if fix is None:
            return src, None
        return src, {'lat': fix.lat, 'lng': fix.lng, 'hdg': fix.hdg}


def source_change_message(old, new):
    """Texto para el log cuando cambia la fuente (None si no cambio)."""
    if old == new:
        return None
    texts = {
        SOURCE_FUSED: 'robot/position: posicion FUSIONADA (TF map->base_footprint + toLL de navsat_transform)',
        SOURCE_GPS_RAW: 'robot/position: sin TF map->base_footprint o sin toLL; se usa el GPS CRUDO de gpsd',
        SOURCE_NONE: 'robot/position: sin posicion (ni TF fusionada ni fix de gpsd)',
    }
    return texts[new]
