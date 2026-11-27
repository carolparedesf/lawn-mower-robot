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


DIAG_REPEAT_S = 30.0        # mientras falte la posicion fusionada, el motivo se repite en el log cada tanto
GPS_FIX_TOPIC = 'gps/fix'   # lo que le llega a navsat_transform (gps_node ya filtrado)

# Motivos por los que falta la posicion fusionada (clave del motivo; el texto lleva el detalle)
FUSED_NO_MODULES = 'sin_modulos'
FUSED_TF_MISSING = 'tf_falta'
FUSED_TF_STALE = 'tf_vieja'
FUSED_TOLL_NOT_READY = 'toll_no_disponible'
FUSED_TOLL_ZERO = 'toll_cero'
FUSED_TOLL_TIMEOUT = 'toll_sin_respuesta'
FUSED_TOLL_ERROR = 'toll_error'
FUSED_OK = 'ok'

# Motivos por los que falta el GPS crudo de gpsd
RAW_WAITING = 'esperando'
RAW_GPSD_DOWN = 'gpsd_caido'
RAW_NOT_ACTIVE = 'gpsd_inactivo'
RAW_NO_FIX = 'sin_fix'
RAW_BAD_REPLY = 'respuesta_incompleta'
RAW_OK = 'ok'


def classify_gpsd_error(exc):
    """(motivo, texto, reconectar) para una excepcion de gpsd.get_current() (gpsd-py3 0.3).

    gpsd-py3 levanta UserWarning si ningun receptor esta activo, IndexError/KeyError si el POLL llega sin TPV o SKY
    (pasa al arrancar el receptor o si gpsd perdio un reporte), Exception('Unexpected message...') si el stream quedo
    desfasado (hay que reconectar) y OSError/ValueError si se corto el socket.
    """
    name = type(exc).__name__
    if isinstance(exc, (OSError, ValueError)):
        return RAW_GPSD_DOWN, f'se corto la conexion con gpsd ({name}: {exc})', True
    if isinstance(exc, UserWarning):
        return RAW_NOT_ACTIVE, 'gpsd no tiene ningun receptor activo (POLL con active=0)', False
    if isinstance(exc, (IndexError, KeyError)):
        return RAW_BAD_REPLY, f'POLL de gpsd sin TPV o SKY ({name}: {exc})', False
    if 'Unexpected message' in str(exc):
        return RAW_GPSD_DOWN, f'respuesta de gpsd fuera de orden ({exc}); se reconecta', True
    return RAW_BAD_REPLY, f'error leyendo gpsd ({name}: {exc})', False


class Diagnosis:
    """Por que robot/position no es la posicion fusionada, para el log.

    Guarda el ultimo motivo de cada fuente (clave, texto) y cuando llego el ultimo /gps/fix (que es lo que necesita
    navsat_transform para tener su transformacion). report() devuelve el texto a loguear: cuando cambia la fuente o
    el motivo, y cada DIAG_REPEAT_S mientras siga faltando la fusionada.
    """

    def __init__(self, now, repeat_s=DIAG_REPEAT_S):
        self.repeat_s = repeat_s
        self.started = now
        self.fused = (FUSED_TOLL_NOT_READY, 'todavia no se pidio ninguna conversion a toLL')
        self.raw = (RAW_WAITING, 'todavia sin respuesta de gpsd')
        self.gps_fix_t = None
        self.gps_fix_ok = None
        self.last_key = None
        self.last_log = -math.inf

    def set_fused(self, key, text=''):
        self.fused = (key, text)

    def set_raw(self, key, text=''):
        self.raw = (key, text)

    def on_gps_fix(self, has_fix, now):
        self.gps_fix_t = now
        self.gps_fix_ok = bool(has_fix)

    def gps_fix_text(self, now):
        if self.gps_fix_t is None:
            return (f'en /{GPS_FIX_TOPIC} no se recibio nada en {now - self.started:.0f} s: gps_node no publica '
                    '(descarta los fixes por calidad, ver su log "fixes descartados", o no lee gpsd)')
        age = now - self.gps_fix_t
        state = 'con fix' if self.gps_fix_ok else 'SIN fix (STATUS_NO_FIX)'
        return f'el ultimo /{GPS_FIX_TOPIC} llego hace {age:.0f} s, {state}'

    def fused_text(self, now):
        key, text = self.fused
        if key == FUSED_TOLL_ZERO:
            return f'{text}; {self.gps_fix_text(now)}'
        return text

    def raw_text(self, selector, now):
        key, text = self.raw
        last = 'nunca hubo fix crudo' if selector.raw is None else f'ultimo fix crudo hace {now - selector.raw.t:.0f} s'
        if key == RAW_OK:
            return f'{last} (gpsd responde con fix: el hilo de gpsd se atraso)'
        return f'{last}; {text}'

    def report(self, source, selector, now):
        """Texto para el log o None. Con la fusionada vigente no hay nada que explicar."""
        if source == SOURCE_FUSED:
            self.last_key = None
            return None
        raw_key = self.raw[0] if source == SOURCE_NONE else None
        key = (source, self.fused[0], raw_key)
        if key == self.last_key and now - self.last_log < self.repeat_s:
            return None
        self.last_key, self.last_log = key, now
        text = f'robot/position sin posicion FUSIONADA: {self.fused_text(now)}.'
        if source == SOURCE_GPS_RAW:
            return text + ' Se publica el GPS crudo de gpsd.'
        return text + f' Tampoco hay GPS crudo: {self.raw_text(selector, now)}.'


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
