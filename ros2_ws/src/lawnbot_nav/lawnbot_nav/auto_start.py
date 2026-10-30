"""Decision de auto-inicio del seguidor al entrar a un modo automatico (sin ROS)."""
from dataclasses import dataclass
from typing import Optional

START = 'start'
WAIT = 'wait'
BLOCKED = 'blocked'
TIMEOUT = 'timeout'
IDLE = 'idle'


@dataclass
class AutoStartInputs:
    follower_idle: bool       # el seguidor puede arrancar (estado IDLE)
    has_route: bool
    mode_ok: bool
    estop: bool
    pose_ok: bool
    geofence_required: bool
    geofence_status: str      # 'OK', 'NO_POLYGON', ...
    geofence_inside: bool


class AutoStart:
    """Se arma al ENTRAR a un modo de `start_modes` (no al arrancar el nodo ya dentro de uno)."""

    def __init__(self, start_modes, timeout_s: float = 30.0):
        self.start_modes = tuple(start_modes)
        self.timeout_s = timeout_s
        self.prev_mode: Optional[str] = None
        self.pending = False
        self.deadline = 0.0

    def on_mode(self, mode: str, now: float) -> bool:
        """Devuelve True si el modo acaba de cambiar a uno de arranque automatico."""
        prev, self.prev_mode = self.prev_mode, mode
        if mode not in self.start_modes:
            self.pending = False
            return False
        if prev is None or prev == mode:     # primer mensaje: solo toma la referencia
            return False
        self.pending = True
        self.deadline = now + self.timeout_s
        return True

    def evaluate(self, now: float, i: AutoStartInputs):
        """Devuelve (accion, alerta). Alertas: geofence_not_ready, auto_start_timeout, ..."""
        if not self.pending:
            return IDLE, None
        if now > self.deadline:
            self.pending = False
            return TIMEOUT, 'auto_start_timeout'
        if not i.follower_idle:
            return WAIT, None
        if not i.has_route:
            return WAIT, None                # la ruta todavia se esta generando
        if i.estop:
            return BLOCKED, 'estop'
        if not i.mode_ok or not i.pose_ok:
            return BLOCKED, 'no_pose' if i.mode_ok else None
        if i.geofence_required and not (i.geofence_status == 'OK' and i.geofence_inside):
            return BLOCKED, 'geofence_not_ready'
        self.pending = False
        return START, None
