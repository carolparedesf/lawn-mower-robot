"""Seguimiento de borde (pared, cerca, cantero) con el sonar y el bumper del lado elegido. Sin ROS.

El sonar del lado de la pared mide la distancia lateral (con su angulo de montaje) y un control
proporcional la mantiene en `target_distance`. El bumper lateral es contacto: si toca, se aleja.
Una esquina interior (obstaculo al frente) se resuelve girando en el lugar hacia el lado contrario
de la pared; una esquina exterior (la pared desaparece) con una busqueda acotada: arco hacia la pared
y tramo recto, por un tiempo maximo. Si no la reencuentra, FAILED y alerta.
"""
import math
from dataclasses import dataclass
from typing import Optional

SEARCH_FORWARD = 'SEARCH_FORWARD'
SEARCH_ARC = 'SEARCH_ARC'
FOLLOW = 'FOLLOW'
CORNER = 'CORNER'
CONTACT_BACK = 'CONTACT_BACK'
CONTACT_TURN = 'CONTACT_TURN'
FAILED = 'FAILED'


@dataclass
class PerimeterParams:
    side: str = 'right'                  # lado de la pared: 'left' o 'right'
    linear: float = 0.15                 # m/s
    max_angular: float = 0.5             # rad/s
    target_distance: float = 0.30        # m, distancia lateral sonar-pared
    kp: float = 1.5                      # rad/s por metro de error
    side_sonar_angle_deg: float = 45.0   # angulo del sonar lateral respecto del frente (0 = mira al frente)
    use_side_sonar: bool = True          # False: solo contacto con el bumper lateral
    max_range: float = 1.5               # lectura del sonar sin eco
    lost_distance: float = 0.9           # pared mas lejos que esto = perdida
    obstacle_distance: float = 0.45      # frente (sonar central) que dispara la esquina interior
    contact_turn: float = 0.5            # rad/s para separarse al tocar el bumper
    contact_back_speed: float = 0.1      # m/s: primero retrocede derecho, porque girar pegado a la pared
    contact_back_time_s: float = 0.8     # barre las esquinas del cuerpo contra ella
    contact_away_time_s: float = 1.2     # despues se aleja girando
    contact_away_linear: float = 0.08
    seek_turn: float = 0.15              # rad/s hacia la pared cuando solo se usa el bumper
    corner_turn_speed: float = 0.6
    corner_clear_ratio: float = 1.3      # el frente se considera libre por encima de obstacle_distance * ratio
    corner_timeout_s: float = 8.0
    lost_timeout_s: float = 0.6          # minimo sin pared ni contacto antes de buscar
    sonar_forward_offset: float = 0.19   # m, el sonar esta adelante del centro de giro
    outer_corner_extra: float = 0.10     # m de avance extra antes de rodear una esquina exterior
    contact_timeout_s: float = 6.0       # contacto continuo con el bumper lateral por mas de esto = bloqueado
    bump_lost_timeout_s: float = 3.0     # idem, en modo solo-bumper
    search_arc_time_s: float = 4.0
    search_arc_linear: float = 0.1
    search_arc_turn: float = 0.4
    search_forward_time_s: float = 3.0
    search_timeout_s: float = 25.0       # tiempo total de busqueda antes de rendirse


@dataclass
class PerimeterInputs:
    sensors_ok: bool
    side_range: Optional[float] = None       # sonar del lado de la pared (None sin dato)
    center_range: Optional[float] = None
    side_bumper: bool = False
    front_bumper: bool = False
    back_bumper: bool = False


class PerimeterCore:
    def __init__(self, params: PerimeterParams):
        self.p = params
        self.sign = 1.0 if params.side == 'left' else -1.0      # +1: la pared esta a la izquierda
        self.reset(0.0)

    def reset(self, now: float) -> None:
        self.state = SEARCH_FORWARD
        self.alert: Optional[str] = None
        self.fail_reason = ''
        self.state_since = now
        self.search_since = now
        self.search_first = 'forward'        # al empezar: derecho; al perder la pared: arco
        self.lost_since: Optional[float] = None
        self.last_lateral = self.p.target_distance
        self.contact_since: Optional[float] = None
        self.lateral: Optional[float] = None

    def config_error(self) -> Optional[str]:
        if self.p.use_side_sonar and self.p.side_sonar_angle_deg < 20.0:
            return 'perimeter_misconfigured'
        return None

    # ── Ciclo ────────────────────────────────────────────────

    def update(self, inp: PerimeterInputs, now: float):
        """Devuelve (v, w). `self.alert` queda con la alerta vigente."""
        p = self.p
        self.alert = None
        err = self.config_error()
        if err:
            return self._fail(err, now)
        if self.state == FAILED:
            self.alert = self.fail_reason
            return 0.0, 0.0
        if not inp.sensors_ok:
            self.alert = 'sensor_timeout'
            return 0.0, 0.0

        self.lateral = self._lateral(inp)
        wall = self.lateral is not None and self.lateral < p.lost_distance
        if wall:
            self.last_lateral = self.lateral
        contact = inp.side_bumper
        if contact:
            if self.contact_since is None:
                self.contact_since = now
            elif now - self.contact_since > p.contact_timeout_s:
                return self._fail('perimeter_blocked', now)
        else:
            self.contact_since = None
        front_blocked = inp.front_bumper or (
            inp.center_range is not None and inp.center_range < p.obstacle_distance)

        if self.state == CORNER:
            clear = (not inp.front_bumper and (inp.center_range is None or
                                                inp.center_range > p.obstacle_distance * p.corner_clear_ratio))
            if clear:
                self._set(FOLLOW, now)
                self.lost_since = None
            elif now - self.state_since > p.corner_timeout_s:
                return self._fail('perimeter_blocked', now)
            else:
                return 0.0, -self.sign * p.corner_turn_speed

        if self.state == CONTACT_BACK:
            if now - self.state_since >= p.contact_back_time_s or inp.back_bumper:
                self._set(CONTACT_TURN, now)
            else:
                return -p.contact_back_speed, 0.0
        if self.state == CONTACT_TURN:
            if now - self.state_since >= p.contact_away_time_s:
                self._set(FOLLOW, now)
                self.lost_since = None
            else:
                return p.contact_away_linear, -self.sign * p.contact_turn

        if front_blocked and self.state in (FOLLOW, SEARCH_FORWARD, SEARCH_ARC):
            self._set(CORNER, now)
            return 0.0, -self.sign * p.corner_turn_speed

        if self.state in (SEARCH_FORWARD, SEARCH_ARC):
            if wall or contact:
                self._set(FOLLOW, now)
                self.lost_since = None
            else:
                return self._search(now)

        # FOLLOW
        if wall or contact:
            self.lost_since = None
        else:
            if self.lost_since is None:
                self.lost_since = now
            if now - self.lost_since >= self._lost_limit():
                self.search_first = 'arc'
                self.search_since = now
                self._set(SEARCH_ARC, now)
                return self._search(now)

        if contact:
            self._set(CONTACT_BACK, now)
            return -p.contact_back_speed, 0.0
        if self.lateral is not None and p.use_side_sonar:
            error = self.lateral - p.target_distance
            w = max(-p.max_angular, min(p.max_angular, self.sign * p.kp * error))
            v = p.linear * max(0.4, 1.0 - 0.6 * abs(w) / p.max_angular)
            return v, w
        # Sin sonar lateral util: se busca el contacto con una curva suave hacia la pared
        return p.linear, self.sign * p.seek_turn

    # ── Internos ─────────────────────────────────────────────

    def _lost_limit(self) -> float:
        """Tiempo que se sigue derecho tras perder la pared antes de rodear la esquina.

        El sonar lateral apunta hacia adelante: ve el final de la pared `lateral / tan(angulo)` antes de
        llegar a la esquina, y esta adelantado respecto del centro de giro.
        """
        p = self.p
        if not p.use_side_sonar:
            return p.bump_lost_timeout_s
        ahead = self.last_lateral / math.tan(math.radians(p.side_sonar_angle_deg)) \
            + p.sonar_forward_offset + p.outer_corner_extra
        return max(p.lost_timeout_s, ahead / p.linear)

    def _lateral(self, inp: PerimeterInputs) -> Optional[float]:
        if not self.p.use_side_sonar or inp.side_range is None:
            return None
        if inp.side_range >= self.p.max_range * 0.98:
            return None                                        # sin eco
        return inp.side_range * math.sin(math.radians(self.p.side_sonar_angle_deg))

    def _search(self, now: float):
        """Busqueda acotada: tramos rectos y arcos hacia la pared, hasta search_timeout_s."""
        p = self.p
        rel = now - self.search_since
        if rel > p.search_timeout_s:
            return self._fail('perimeter_lost', now)
        phase = rel % (p.search_arc_time_s + p.search_forward_time_s)
        if self.search_first == 'arc':
            arc = phase < p.search_arc_time_s
        else:
            arc = phase >= p.search_forward_time_s
        if arc:
            self.state = SEARCH_ARC
            return p.search_arc_linear, self.sign * p.search_arc_turn
        self.state = SEARCH_FORWARD
        return p.linear, 0.0

    def _fail(self, reason: str, now: float):
        self.state = FAILED
        self.fail_reason = reason
        self.alert = reason
        return 0.0, 0.0

    def _set(self, state: str, now: float) -> None:
        self.state = state
        self.state_since = now
