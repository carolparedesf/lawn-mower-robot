"""Seguimiento de borde (pared, cerca, cantero) por contacto con el bumper lateral. Sin ROS.

Modo base (bumper): avanza con una curva suave hacia la pared hasta tocarla. Al tocar, RETROCEDE derecho
un poco (para que la cola no barra la pared al girar) y despues gira en el lugar alejandose un angulo
fijo; luego vuelve a buscar el contacto. Asi avanza zigzagueando pegado al borde.

Refinamiento opcional (`use_side_sonar`, solo con `side_sonar_angle_deg` valido): el sonar del lado de la
pared mide la distancia lateral y un control proporcional la mantiene, con el bumper como contacto de
respaldo. Si el angulo no es valido, el sonar se ignora y se sigue por bumper.

Esquina interior: el sonar central o el bumper frontal disparan el giro. Esquina exterior / contacto
perdido: busqueda acotada y, si falla, FAILED con alerta.
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
CONTACT_FWD = 'CONTACT_FWD'
UNSTICK = 'UNSTICK'
FAILED = 'FAILED'

MIN_SONAR_ANGLE_DEG = 20.0


@dataclass
class PerimeterParams:
    side: str = 'right'                  # lado de la pared: 'left' o 'right'
    linear: float = 0.15                 # m/s
    max_angular: float = 0.5             # rad/s

    # ── Contacto con el bumper (modo base) ──
    seek_turn: float = 0.3               # rad/s: curva hacia la pared mientras no hay contacto
    contact_back_speed: float = 0.10     # m/s: retroceso recto al tocar (la cola no barre la pared)
    contact_back_distance: float = 0.10  # m
    contact_turn_angle_deg: float = 15.0         # giro en el lugar para alejarse tras un contacto lateral
    front_contact_turn_angle_deg: float = 45.0   # idem tras un contacto frontal (esquina interior)
    contact_turn_speed: float = 0.5      # rad/s
    # Si al girar el bumper sigue presionado (la cola roza la pared), avanza un poco en vez de empujar
    contact_unwedge_after_s: float = 1.0
    contact_forward_speed: float = 0.08  # m/s
    contact_forward_time_s: float = 1.5
    contact_timeout_s: float = 6.0       # contacto continuo por mas de esto = bloqueado
    bump_lost_timeout_s: float = 10.0    # sin contacto por mas de esto = contacto perdido -> busqueda

    # Si queda trabado (contacto continuo o esquina que no libera), retrocede en arco alejandose de la pared
    # para ganar espacio de giro; tras `unstick_attempts` intentos sin salir, FAILED (perimeter_blocked)
    unstick_attempts: int = 2
    unstick_speed: float = 0.10          # m/s hacia atras
    unstick_turn: float = 0.4            # rad/s del arco
    unstick_time_s: float = 2.5

    # ── Sonar central (esquina interior anticipada) ──
    use_center_sonar: bool = True
    obstacle_distance: float = 0.45
    corner_turn_speed: float = 0.6
    corner_clear_ratio: float = 1.3
    corner_timeout_s: float = 8.0

    # ── Refinamiento opcional con el sonar lateral ──
    use_side_sonar: bool = False
    side_sonar_angle_deg: float = 45.0   # angulo del sonar lateral respecto del frente (0 = mira al frente)
    target_distance: float = 0.30        # m, distancia lateral sonar-pared
    kp: float = 1.5                      # rad/s por metro de error
    max_range: float = 1.5               # lectura del sonar sin eco
    lost_distance: float = 0.9           # pared mas lejos que esto = perdida
    lost_timeout_s: float = 0.6
    sonar_forward_offset: float = 0.19   # m, el sonar esta adelante del centro de giro
    outer_corner_extra: float = 0.10     # m de avance extra antes de rodear una esquina exterior

    # ── Busqueda acotada ──
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

    @property
    def sonar_enabled(self) -> bool:
        """El sonar lateral solo se usa si se pidio y su angulo de montaje es valido."""
        return self.p.use_side_sonar and self.p.side_sonar_angle_deg >= MIN_SONAR_ANGLE_DEG

    def reset(self, now: float) -> None:
        self.state = SEARCH_FORWARD
        self.alert: Optional[str] = None
        self.fail_reason = ''
        self.state_since = now
        self.search_since = now
        self.search_first = 'forward'        # al empezar: derecho; al perder la pared: arco
        self.lost_since: Optional[float] = None
        self.last_contact = now
        self.last_lateral = self.p.target_distance
        self.contact_since: Optional[float] = None
        self.contact_kind = 'side'
        self.unstick_count = 0
        self.lateral: Optional[float] = None

    # ── Ciclo ────────────────────────────────────────────────

    def update(self, inp: PerimeterInputs, now: float):
        """Devuelve (v, w). `self.alert` queda con la alerta vigente."""
        p = self.p
        self.alert = None
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
        contact = inp.side_bumper or inp.front_bumper
        if contact:
            self.last_contact = now
            if self.contact_since is None:
                self.contact_since = now
            elif now - self.contact_since > p.contact_timeout_s and self.state != UNSTICK:
                return self._stuck(now)
        else:
            self.contact_since = None

        if self.state == UNSTICK:
            if now - self.state_since >= p.unstick_time_s or inp.back_bumper:
                self._set(FOLLOW, now)
                self.contact_since = None
                self.lost_since = None
                self.last_contact = now
            else:
                return -p.unstick_speed, self.sign * p.unstick_turn

        # Retroceso y giro tras un contacto
        if self.state == CONTACT_BACK:
            back_time = p.contact_back_distance / p.contact_back_speed
            if now - self.state_since >= back_time or inp.back_bumper:
                self._set(CONTACT_TURN, now)
            else:
                return -p.contact_back_speed, 0.0
        if self.state == CONTACT_TURN:
            angle = (p.front_contact_turn_angle_deg if self.contact_kind == 'front'
                     else p.contact_turn_angle_deg)
            if now - self.state_since >= math.radians(angle) / p.contact_turn_speed:
                self._set(FOLLOW, now)
                self.lost_since = None
                self.last_contact = now
            elif (contact or inp.back_bumper) and now - self.state_since >= p.contact_unwedge_after_s:
                self._set(CONTACT_FWD, now)
                return p.contact_forward_speed, 0.0
            else:
                return 0.0, -self.sign * p.contact_turn_speed
        if self.state == CONTACT_FWD:
            if now - self.state_since >= p.contact_forward_time_s:
                self._set(FOLLOW, now)
                self.lost_since = None
                self.last_contact = now
            else:
                return p.contact_forward_speed, 0.0

        # Esquina interior detectada por el sonar central, antes de tocar
        center_blocked = (p.use_center_sonar and inp.center_range is not None
                          and inp.center_range < p.obstacle_distance)
        if self.state == CORNER:
            clear = (not inp.front_bumper and (inp.center_range is None or
                                                inp.center_range > p.obstacle_distance * p.corner_clear_ratio))
            if clear:
                self._set(FOLLOW, now)
                self.lost_since = None
            elif now - self.state_since > p.corner_timeout_s:
                return self._stuck(now)
            else:
                return 0.0, -self.sign * p.corner_turn_speed
        elif center_blocked and not contact and self.state in (FOLLOW, SEARCH_FORWARD, SEARCH_ARC):
            self.last_contact = now                       # una pared al frente tambien cuenta como borde encontrado
            self._set(CORNER, now)
            return 0.0, -self.sign * p.corner_turn_speed

        # Contacto: retrocede y se aleja. Si toca la cola (bumper trasero) no se puede retroceder:
        # al reves, se avanza para liberarla.
        if (contact or inp.back_bumper) and self.state in (FOLLOW, SEARCH_FORWARD, SEARCH_ARC):
            self.contact_kind = 'front' if inp.front_bumper else 'side'
            if inp.back_bumper:
                self._set(CONTACT_FWD, now)
                return p.contact_forward_speed, 0.0
            self._set(CONTACT_BACK, now)
            return -p.contact_back_speed, 0.0

        # Busqueda: se sale al encontrar la pared (por contacto ya se atendio arriba)
        if self.state in (SEARCH_FORWARD, SEARCH_ARC):
            if wall:
                self._set(FOLLOW, now)
                self.lost_since = None
            else:
                return self._search(now)

        # FOLLOW
        if self.state == FOLLOW and self.unstick_count and now - self.state_since > 20.0:
            self.unstick_count = 0                        # 20 s en FOLLOW sin trabarse: se renuevan los intentos
        if self.sonar_enabled:
            if wall:
                self.lost_since = None
            else:
                if self.lost_since is None:
                    self.lost_since = now
                if now - self.lost_since >= self._lost_limit():
                    return self._start_search(now)
            if self.lateral is not None and wall:
                error = self.lateral - p.target_distance
                w = max(-p.max_angular, min(p.max_angular, self.sign * p.kp * error))
                v = p.linear * max(0.4, 1.0 - 0.6 * abs(w) / p.max_angular)
                return v, w
            return p.linear, 0.0                      # esquina exterior: sigue derecho hasta rodearla
        if now - self.last_contact >= p.bump_lost_timeout_s:
            return self._start_search(now)
        return p.linear, self.sign * p.seek_turn       # curva suave hacia la pared hasta tocarla

    # ── Internos ─────────────────────────────────────────────

    def _stuck(self, now: float):
        """Trabado: intenta liberarse retrocediendo en arco; si ya lo intento varias veces, se rinde."""
        if self.unstick_count >= self.p.unstick_attempts:
            return self._fail('perimeter_blocked', now)
        self.unstick_count += 1
        self._set(UNSTICK, now)
        return -self.p.unstick_speed, self.sign * self.p.unstick_turn

    def _start_search(self, now: float):
        self.search_first = 'arc'
        self.search_since = now
        self._set(SEARCH_ARC, now)
        return self._search(now)

    def _lost_limit(self) -> float:
        """Tiempo que se sigue derecho tras perder la pared antes de rodear la esquina (solo con sonar).

        El sonar lateral apunta hacia adelante: ve el final de la pared `lateral / tan(angulo)` antes de
        llegar a la esquina, y esta adelantado respecto del centro de giro.
        """
        p = self.p
        ahead = self.last_lateral / math.tan(math.radians(p.side_sonar_angle_deg)) \
            + p.sonar_forward_offset + p.outer_corner_extra
        return max(p.lost_timeout_s, ahead / p.linear)

    def _lateral(self, inp: PerimeterInputs) -> Optional[float]:
        if not self.sonar_enabled or inp.side_range is None:
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
