"""Seguidor de waypoints (pure pursuit) sin dependencias de ROS.

Maquina de estados + controlador. El nodo waypoint_follower solo aporta pose (TF), entradas
(enable, modo, estop, manual) y publica el resultado; todo lo demas se prueba aqui.
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple

IDLE = 'IDLE'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
DONE = 'DONE'
ERROR = 'ERROR'


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class FollowerParams:
    waypoints: List[Tuple[float, float]] = field(default_factory=list)
    relative_to_start: bool = True
    loop: bool = False
    lookahead_distance: float = 0.5
    max_linear: float = 0.2
    min_linear: float = 0.0          # piso de v al avanzar (zona muerta del PWM); 0 = sin piso
    max_angular: float = 0.6
    min_turn: float = 0.0            # piso de w al girar en el lugar; 0 = sin piso
    goal_tolerance: float = 0.2
    rotate_in_place_angle: float = 0.7   # |error de rumbo| que dispara el giro en el lugar [rad]
    rotate_exit_ratio: float = 0.5       # histeresis: sale del giro por debajo de angle*ratio
    turn_gain: float = 1.5
    slowdown_radius: float = 0.6
    progress_timeout: float = 20.0       # 0 = desactivado
    progress_distance_eps: float = 0.03  # [m] de acercamiento que cuenta como avance
    progress_heading_eps: float = 0.03   # [rad] de reduccion de error de rumbo que cuenta como avance
    tf_abort_s: float = 3.0              # sin pose por mas de esto -> ERROR
    # Tolerancia efectiva de la ruta: min(goal_tolerance, ratio * tramo mas corto). Evita saltar
    # la vuelta entre lineas de cobertura cuando el espaciado es parecido a la tolerancia.
    route_tolerance_ratio: float = 0.4
    min_route_tolerance: float = 0.05


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class WaypointFollowerCore:
    def __init__(self, params: FollowerParams):
        self.p = params
        self.state = IDLE
        self.reason = ''
        # Un True de enable solo se acepta si esta armado: se desarma al pausar por manual,
        # estop o modo, y se vuelve a armar con un False.
        self.armed = True
        self.wps: List[Tuple[float, float]] = []
        self.route = None            # ruta absoluta recibida por topic (reemplaza a p.waypoints)
        self.tol = params.goal_tolerance
        self.k = 0
        self.laps = 0
        self.seg_start = (0.0, 0.0)
        self.rotating = False
        self.lost_since = None
        self.info = {}
        self._reset_progress(0.0)

    # ── Comandos externos ────────────────────────────────────

    def enable(self, pose: Pose, now: float) -> str:
        if not self.armed:
            return 'enable ignorado: publicar false y luego true para volver a habilitar'
        if self.state == RUNNING:
            return 'ya estaba en RUNNING (sin cambios)'
        if not self.p.waypoints and not self.route:
            return 'sin waypoints configurados'
        if self.state == PAUSED:
            self.state = RUNNING
            self.reason = ''
            self.lost_since = None
            self._reset_progress(now)
            return 'reanudado'
        # IDLE, DONE o ERROR: corrida nueva
        self._anchor(pose)
        self.tol = self._effective_tolerance()
        self.k = 0
        self.laps = 0
        self.seg_start = (pose.x, pose.y)
        self.rotating = False
        self.lost_since = None
        self.state = RUNNING
        self.reason = ''
        self._reset_progress(now)
        return 'iniciado'

    def disable(self) -> None:
        """enable=false: pausa y deja el enable armado para el proximo true."""
        self.armed = True
        if self.state == RUNNING:
            self.state = PAUSED
            self.reason = 'enable=false'

    def rearm(self) -> None:
        """Vuelve a aceptar `enable` (p. ej. al elegir de nuevo un modo automatico)."""
        self.armed = True

    def set_route(self, points) -> str:
        """Carga una ruta absoluta (en el frame del seguidor). Rechazada si esta en RUNNING."""
        if self.state == RUNNING:
            return 'ruta rechazada: el seguidor esta en RUNNING'
        pts = []
        for pt in points:
            if not pts or math.dist(pts[-1], pt) > 1e-6:
                pts.append((float(pt[0]), float(pt[1])))
        if not pts:
            self.clear_route()
            return 'ruta vacia: borrada'
        self.route = pts
        self.wps = list(pts)
        self.tol = self._effective_tolerance()
        self.k = 0
        self.state = IDLE
        self.reason = ''
        return f'ruta cargada: {len(pts)} puntos, tolerancia efectiva {self.tol:.3f} m'

    def clear_route(self) -> None:
        """Borra la ruta recibida; si estaba navegando pasa a IDLE (el nodo publica cero)."""
        self.route = None
        self.wps = []
        self.k = 0
        self.tol = self.p.goal_tolerance
        self.state = IDLE
        self.reason = ''

    def pause(self, reason: str) -> None:
        """Pausa externa (manual, estop, modo): exige false y luego true para reanudar."""
        if self.state == RUNNING:
            self.state = PAUSED
            self.reason = reason
            self.armed = False

    # ── Ciclo de control ─────────────────────────────────────

    def pose_unavailable(self, now: float):
        """Llamar en cada ciclo sin pose valida. Devuelve (0, 0)."""
        if self.state != RUNNING:
            return 0.0, 0.0
        if self.lost_since is None:
            self.lost_since = now
        self.progress_t = now   # el tiempo sin pose no cuenta como falta de avance
        if now - self.lost_since > self.p.tf_abort_s:
            self.state = ERROR
            self.reason = 'tf_lost'
        return 0.0, 0.0

    def update(self, pose: Pose, now: float):
        """Devuelve (v, w). Cero salvo en RUNNING."""
        if self.state != RUNNING:
            return 0.0, 0.0
        self.lost_since = None
        p = self.p
        pos = (pose.x, pose.y)

        # Avanzar de waypoint al entrar en la tolerancia
        for _ in range(len(self.wps) + 1):
            goal = self.wps[self.k]
            if math.dist(pos, goal) >= self.tol:
                break
            self.seg_start = goal
            self.k += 1
            self._reset_progress(now)
            self.rotating = False
            if self.k >= len(self.wps):
                if p.loop:
                    self.k = 0
                    self.laps += 1
                else:
                    self.state = DONE
                    self.reason = 'completed'
                    self.info = {'k': len(self.wps), 'n': len(self.wps), 'dist': 0.0,
                                 'alpha': 0.0, 'v': 0.0, 'w': 0.0, 'phase': 'done'}
                    return 0.0, 0.0

        goal = self.wps[self.k]
        target = self._lookahead_point(pos, goal)
        dx, dy = target[0] - pos[0], target[1] - pos[1]
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        xr = c * dx + s * dy
        yr = -s * dx + c * dy
        ld = max(math.hypot(dx, dy), 1e-3)
        alpha = math.atan2(yr, xr)
        d_goal = math.dist(pos, goal)

        # Giro en el lugar con histeresis
        if self.rotating:
            if abs(alpha) < p.rotate_in_place_angle * p.rotate_exit_ratio:
                self.rotating = False
        elif abs(alpha) > p.rotate_in_place_angle:
            self.rotating = True

        if self.rotating:
            mag = min(max(p.turn_gain * abs(alpha), p.min_turn), p.max_angular)
            v, w = 0.0, math.copysign(mag, alpha)
            phase = 'rotate'
        else:
            kappa = 2.0 * yr / (ld * ld)
            v = p.max_linear * max(0.0, math.cos(alpha))
            if p.slowdown_radius > 0.0:
                v *= min(1.0, d_goal / p.slowdown_radius)
            if abs(kappa) > 1e-9 and v * abs(kappa) > p.max_angular:
                v = p.max_angular / abs(kappa)
            if p.min_linear > 0.0:
                v = max(v, min(p.min_linear, p.max_linear))
            v = min(v, p.max_linear)
            w = clamp(v * kappa, -p.max_angular, p.max_angular)
            phase = 'drive'

        self._track_progress(phase, d_goal, abs(alpha), now)
        if p.progress_timeout > 0.0 and now - self.progress_t > p.progress_timeout:
            self.state = ERROR
            self.reason = 'no_progress'
            v = w = 0.0

        self.info = {'k': self.k, 'n': len(self.wps), 'dist': d_goal, 'alpha': alpha,
                     'v': v, 'w': w, 'phase': phase, 'tol': self.tol}
        return v, w

    # ── Internos ─────────────────────────────────────────────

    def _anchor(self, pose: Pose) -> None:
        if self.route is not None:          # las rutas por topic siempre son absolutas
            self.wps = list(self.route)
        elif self.p.relative_to_start:
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            self.wps = [(pose.x + c * x - s * y, pose.y + s * x + c * y)
                        for x, y in self.p.waypoints]
        else:
            self.wps = list(self.p.waypoints)

    def _effective_tolerance(self) -> float:
        wps = self.wps
        legs = [math.dist(a, b) for a, b in zip(wps, wps[1:])]
        if self.p.loop and len(wps) > 1:
            legs.append(math.dist(wps[-1], wps[0]))
        legs = [d for d in legs if d > 1e-6]
        if not legs:
            return self.p.goal_tolerance
        return min(self.p.goal_tolerance,
                   max(self.p.min_route_tolerance, self.p.route_tolerance_ratio * min(legs)))

    def _lookahead_point(self, pos, goal):
        a = self.seg_start
        abx, aby = goal[0] - a[0], goal[1] - a[1]
        length2 = abx * abx + aby * aby
        if length2 < 1e-9:
            return goal
        t = clamp(((pos[0] - a[0]) * abx + (pos[1] - a[1]) * aby) / length2, 0.0, 1.0)
        proj = (a[0] + t * abx, a[1] + t * aby)
        if math.dist(proj, goal) <= self.p.lookahead_distance:
            return goal
        length = math.sqrt(length2)
        return (proj[0] + abx / length * self.p.lookahead_distance,
                proj[1] + aby / length * self.p.lookahead_distance)

    def _reset_progress(self, now: float) -> None:
        self.phase = None
        self.best_dist = math.inf
        self.best_alpha = math.inf
        self.progress_t = now

    def _track_progress(self, phase, d_goal, abs_alpha, now) -> None:
        """Avanzar = acercarse al waypoint (al conducir) o reducir el error de rumbo (al girar)."""
        if phase != self.phase:
            self.phase = phase
            self.best_dist = d_goal
            self.best_alpha = abs_alpha
            self.progress_t = now
        elif phase == 'drive':
            if d_goal < self.best_dist - self.p.progress_distance_eps:
                self.best_dist = d_goal
                self.progress_t = now
        else:
            if abs_alpha < self.best_alpha - self.p.progress_heading_eps:
                self.best_alpha = abs_alpha
                self.progress_t = now
