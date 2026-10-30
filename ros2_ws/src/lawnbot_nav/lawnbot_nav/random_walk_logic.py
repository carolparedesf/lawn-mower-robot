"""Caminata aleatoria: AVANZAR -> (obstaculo, bumper o borde del cerco) -> RETROCEDER -> GIRAR -> AVANZAR.

Sin ROS. Los giros son por tiempo (sin realimentacion de rumbo): para una caminata al azar alcanza.
"""
import random
from collections import deque
from dataclasses import dataclass
from typing import Optional

FORWARD = 'FORWARD'
BACKUP = 'BACKUP'
TURN = 'TURN'
BLOCKED = 'BLOCKED'


@dataclass
class RandomWalkParams:
    linear: float = 0.2                 # m/s
    backup_speed: float = 0.15          # m/s
    backup_distance: float = 0.35       # m
    turn_speed: float = 0.6             # rad/s
    turn_min_deg: float = 60.0
    turn_max_deg: float = 180.0
    edge_turn_min_deg: float = 120.0    # al salir de un borde del cerco se gira mas, para alejarse
    edge_turn_max_deg: float = 180.0
    obstacle_distance: float = 0.50     # sonar [m]; por debajo se considera obstaculo
    edge_distance: float = 1.5          # distancia al borde del cerco [m]; mayor que el error del GPS
    edge_hard_distance: float = 0.5     # por debajo de esto el borde dispara siempre, aunque no se acerque
    edge_trend_window_s: float = 1.0    # dentro de la franja solo dispara si se esta acercando al borde
    edge_trend_eps: float = 0.1         # [m] de acercamiento en la ventana que cuenta como "acercandose"
    require_geofence: bool = True       # sin cerco OK el robot no se mueve
    stuck_triggers: int = 6             # tantos disparos en stuck_window_s = atrapado
    stuck_window_s: float = 30.0


@dataclass
class RandomWalkInputs:
    sonar_min: Optional[float]          # minimo de los sonares habilitados (None si falta alguno)
    sensors_ok: bool                    # todos los sensores requeridos con dato vigente
    front_bumper: bool = False
    side_bumper: bool = False
    back_bumper: bool = False
    edge_distance: Optional[float] = None   # /geofence/distance_to_edge (None si NaN o sin dato)
    geofence_ok: bool = False               # /geofence/status == OK


class RandomWalkCore:
    def __init__(self, params: RandomWalkParams, seed: Optional[int] = None):
        self.p = params
        self.rng = random.Random(seed)
        self.reset(0.0)

    def reset(self, now: float) -> None:
        self.state = FORWARD
        self.reason = ''
        self.alert: Optional[str] = None
        self.until = 0.0
        self.turn_sign = 1.0
        self.triggers = deque()
        self.edge_history = deque()
        self.n_backups = 0

    def update(self, inp: RandomWalkInputs, now: float):
        """Devuelve (v, w). `self.alert` queda con la alerta vigente (None si no hay)."""
        p = self.p
        self.alert = None

        if not inp.sensors_ok:
            return self._block('sensor_timeout')
        if p.require_geofence and not (inp.geofence_ok and inp.edge_distance is not None):
            return self._block('geofence_not_ready')
        if self.state == BLOCKED:
            self.state = FORWARD

        if len(self.triggers) >= p.stuck_triggers and now - self.triggers[0] <= p.stuck_window_s:
            self.alert = 'random_blocked'

        if self.state == FORWARD:
            reason = self._trigger(inp, now)
            if reason is None:
                return p.linear, 0.0
            self.triggers.append(now)
            while self.triggers and now - self.triggers[0] > p.stuck_window_s:
                self.triggers.popleft()
            self.reason = reason
            self.n_backups += 1
            if inp.back_bumper:
                self._start_turn(now)
                return 0.0, self.turn_sign * p.turn_speed
            self.state = BACKUP
            self.until = now + p.backup_distance / p.backup_speed
            return -p.backup_speed, 0.0

        if self.state == BACKUP:
            if inp.back_bumper or now >= self.until:
                self._start_turn(now)
                return 0.0, self.turn_sign * p.turn_speed
            return -p.backup_speed, 0.0

        # TURN
        if now >= self.until:
            self.state = FORWARD
            return p.linear, 0.0
        return 0.0, self.turn_sign * p.turn_speed

    # ── Internos ─────────────────────────────────────────────

    def _trigger(self, inp: RandomWalkInputs, now: float) -> Optional[str]:
        if inp.front_bumper or inp.side_bumper:
            return 'bumper'
        if inp.sonar_min is not None and inp.sonar_min < self.p.obstacle_distance:
            return 'obstacle'
        if inp.edge_distance is not None and self._edge_trigger(inp.edge_distance, now):
            return 'edge'
        return None

    def _edge_trigger(self, edge: float, now: float) -> bool:
        """Dentro de la franja del borde solo dispara si se acerca (o ya esta demasiado cerca).

        Asi puede arrancar cerca del borde sin quedar en un bucle de retrocesos, y sale hacia adentro.
        """
        p = self.p
        hist = self.edge_history
        while hist and now - hist[0][0] > p.edge_trend_window_s:
            hist.popleft()
        reference = hist[0][1] if hist and now - hist[0][0] >= 0.5 * p.edge_trend_window_s else None
        hist.append((now, edge))
        if edge >= p.edge_distance:
            return False
        if edge < p.edge_hard_distance:
            return True
        return reference is not None and edge < reference - p.edge_trend_eps

    def _start_turn(self, now: float) -> None:
        p = self.p
        lo, hi = ((p.edge_turn_min_deg, p.edge_turn_max_deg) if self.reason == 'edge'
                  else (p.turn_min_deg, p.turn_max_deg))
        angle = self.rng.uniform(lo, hi)
        self.turn_sign = self.rng.choice((-1.0, 1.0))
        self.state = TURN
        self.until = now + (angle * 3.141592653589793 / 180.0) / p.turn_speed

    def _block(self, alert: str):
        self.state = BLOCKED
        self.alert = alert
        return 0.0, 0.0
