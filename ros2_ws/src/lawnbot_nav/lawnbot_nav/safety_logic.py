"""Reglas del safety_gate sin dependencias de ROS (se prueban aparte).

Solo se limita o anula: ningun factor supera 1 ni invierte el signo de la orden.
"""
import math
from dataclasses import dataclass, field
from typing import List

ALERT_NONE = 'none'
ALERT_SENSOR_TIMEOUT = 'sensor_timeout'
ALERT_BUMPER_FRONT = 'bumper_front'
ALERT_BUMPER_BACK = 'bumper_back'
ALERT_OBSTACLE_STOP = 'obstacle_stop'
ALERT_OBSTACLE_NEAR = 'obstacle_near'


@dataclass
class SafetyParams:
    sonar_slow_distance: float = 0.30
    sonar_stop_distance: float = 0.15
    max_linear: float = 0.5
    max_angular: float = 1.5


@dataclass
class SafetyState:
    # Distancias [m] de los sonares habilitados con dato reciente (NaN = lectura invalida)
    sonar_ranges: List[float] = field(default_factory=list)
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

    valid = [d for d in state.sonar_ranges if not math.isnan(d)]
    min_range = min(valid) if valid else None

    forward_factor = 1.0
    if min_range is not None:
        forward_factor = sonar_scale(
            min_range, params.sonar_slow_distance, params.sonar_stop_distance)
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
    elif min_range is not None and min_range <= params.sonar_stop_distance:
        alert = ALERT_OBSTACLE_STOP
    elif min_range is not None and min_range < params.sonar_slow_distance:
        alert = ALERT_OBSTACLE_NEAR
    else:
        alert = ALERT_NONE
    return v, w, alert

