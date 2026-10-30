"""Reglas del safety_gate sin dependencias de ROS (se prueban aparte).

Solo se limita o anula: ningun factor supera 1 ni invierte el signo de la orden.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Union

ALERT_NONE = 'none'
ALERT_SENSOR_TIMEOUT = 'sensor_timeout'
ALERT_BUMPER_FRONT = 'bumper_front'
ALERT_BUMPER_BACK = 'bumper_back'
ALERT_OBSTACLE_STOP = 'obstacle_stop'
ALERT_OBSTACLE_NEAR = 'obstacle_near'

# De mayor a menor prioridad. Las alertas de otros nodos llegan por /nav/alert_in ('fuente:alerta')
ALERT_PRIORITY = (
    ALERT_SENSOR_TIMEOUT, ALERT_BUMPER_FRONT, ALERT_BUMPER_BACK, ALERT_OBSTACLE_STOP,
    'estop', 'geofence_breach', 'tf_lost', 'no_progress', 'perimeter_blocked', 'perimeter_lost', 'perimeter_misconfigured',
    'random_blocked', 'geofence_not_ready', 'coverage_error', 'no_pose',
    'auto_start_timeout', ALERT_OBSTACLE_NEAR,
)


# Sonares que pueden ignorarse en auto_perimeter. El central y los bumpers nunca.
SIDE_SONARS = ('left', 'right')
PERIMETER_MODE = 'auto_perimeter'


@dataclass
class SafetyParams:
    sonar_slow_distance: float = 0.30
    sonar_stop_distance: float = 0.15
    max_linear: float = 0.5
    max_angular: float = 1.5
    # Umbrales propios por sonar: nombre -> (slow, stop). Los que falten usan los globales.
    sonar_thresholds: Dict[str, Tuple[float, float]] = field(default_factory=dict)

    def thresholds(self, name) -> Tuple[float, float]:
        slow, stop = self.sonar_thresholds.get(name, (self.sonar_slow_distance, self.sonar_stop_distance))
        return slow, max(stop, 0.0)


@dataclass
class SafetyState:
    # Distancias [m] de los sonares habilitados con dato reciente (NaN = lectura invalida): lista
    # (umbrales globales) o {nombre: distancia} (umbrales por sonar)
    sonar_ranges: Union[List[float], Dict[str, float]] = field(default_factory=list)
    front_pressed: bool = False
    back_pressed: bool = False
    # Algun sensor requerido (sonar habilitado o bumper frontal) sin datos recientes
    stale: bool = False


def clamp_magnitude(value: float, limit: float) -> float:
    if value == 0.0:
        return 0.0
    return math.copysign(min(abs(value), abs(limit)), value)


def sonar_scale(distance: float, slow: float, stop: float) -> float:
    """1.0 lejos, 0.0 en la zona de parada, lineal entre ambas."""
    if slow <= stop:
        return 0.0 if distance <= stop else 1.0
    if distance >= slow:
        return 1.0
    if distance <= stop:
        return 0.0
    return (distance - stop) / (slow - stop)


def apply_safety(v: float, w: float, state: SafetyState, params: SafetyParams):
    """Devuelve (v, w, alerta). |v_out| <= |v_in| y |w_out| <= |w_in| siempre."""
    v_in, w_in = v, w
    v = clamp_magnitude(v, params.max_linear)
    w = clamp_magnitude(w, params.max_angular)

    readings = (list(state.sonar_ranges.items()) if isinstance(state.sonar_ranges, dict)
                else [(None, d) for d in state.sonar_ranges])
    forward_factor = 1.0
    stop_hit = near_hit = False
    for name, distance in readings:
        if math.isnan(distance):
            continue
        slow, stop = params.thresholds(name)
        forward_factor = min(forward_factor, sonar_scale(distance, slow, stop))
        stop_hit = stop_hit or distance <= stop
        near_hit = near_hit or distance < slow
    if state.front_pressed or state.stale:
        forward_factor = 0.0

    if v > 0.0:
        v *= forward_factor
    elif v < 0.0 and state.back_pressed:
        v = 0.0

    # Garantia final: nunca amplificar
    v = clamp_magnitude(v, abs(v_in))
    w = clamp_magnitude(w, abs(w_in))

    if state.stale:
        alert = ALERT_SENSOR_TIMEOUT
    elif state.front_pressed:
        alert = ALERT_BUMPER_FRONT
    elif state.back_pressed:
        alert = ALERT_BUMPER_BACK
    elif stop_hit:
        alert = ALERT_OBSTACLE_STOP
    elif near_hit:
        alert = ALERT_OBSTACLE_NEAR
    else:
        alert = ALERT_NONE
    return v, w, alert


def ignored_sonars(ignore_sides: bool, mode, mode_fresh: bool) -> Tuple[str, ...]:
    """Sonares que no cuentan en este momento: solo los laterales, solo en auto_perimeter y solo si el
    modo es vigente. El sonar central nunca se ignora."""
    if ignore_sides and mode_fresh and mode == PERIMETER_MODE:
        return SIDE_SONARS
    return ()


def parse_alert_in(text: str):
    """'fuente:alerta' -> (fuente, alerta). Sin ':' la fuente es 'desconocida'."""
    source, sep, alert = text.partition(':')
    if not sep:
        return 'desconocida', source.strip()
    return source.strip(), alert.strip()


def merge_alerts(local_alert: str, external, ttl: float = 3.0, priority=ALERT_PRIORITY) -> str:
    """Alerta unica a publicar: la de mayor prioridad entre la local y las externas vigentes.

    `external` es {fuente: (alerta, edad_en_segundos)}; las mas viejas que `ttl` se ignoran.
    """
    active = [local_alert] if local_alert and local_alert != ALERT_NONE else []
    active += [alert for alert, age in external.values()
               if alert and alert != ALERT_NONE and age <= ttl]
    if not active:
        return ALERT_NONE
    order = {name: i for i, name in enumerate(priority)}
    return min(active, key=lambda a: order.get(a, len(priority)))
