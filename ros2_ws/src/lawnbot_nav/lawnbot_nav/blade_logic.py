"""Condiciones de encendido de los motores de corte (sin ROS). Falla segura: ante cualquier duda, 0.

Cada motor (cuchilla `cutter` y `trimmer`) solo recibe duty > 0 si TODAS estas condiciones se cumplen:
  - fue pedido (enable), - el robot esta dentro del cerco (/geofence/inside), - no esta levantado (bumpers),
  - no esta volcado (IMU), - no hay estop, - la celda tiene menos de `max_passes` pasadas, - todos los datos
  requeridos son recientes, y (solo la cuchilla) - hay pasto (grass_present).
`force_cut` anula UNICAMENTE la condicion de pasto (tambien si el dato falta); nunca el pedido, el cerco, el
levantado, el volcado, el estop, las pasadas ni la vigencia de los demas datos.
Nunca se publica mas que `max_duty` de cada motor.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

BUMPERS = ('bumper/B1_front_left', 'bumper/B2_front_right', 'bumper/B3_left', 'bumper/B4_right', 'bumper/B5_back')
B1, B2, B3, B4, B5 = BUMPERS
# Pares que, presionados a la vez, indican que el robot esta levantado (misma logica que el firmware)
LIFT_PAIRS = ((B3, B4), (B1, B5), (B2, B5))


@dataclass
class BladeParams:
    # Tope del duty publicado, por motor. CONSERVADOR: ajustar con la tension del motor y del pack.
    max_duty_cutter: float = 0.30
    max_duty_trimmer: float = 0.30
    # Si se conocen, el tope se limita ademas a tension_nominal_del_motor / tension_del_pack (0 = desconocida)
    pack_voltage: float = 0.0
    cutter_nominal_voltage: float = 0.0
    trimmer_nominal_voltage: float = 0.0

    max_passes: int = 2                 # la celda debe tener MENOS pasadas que esto
    require_grass: bool = True          # solo la cuchilla
    max_tilt_deg: float = 20.0          # inclinacion (roll/pitch combinados) que se considera volcado
    tilt_hold_s: float = 3.0            # tras volcarse, se mantiene apagado este tiempo
    lift_hold_s: float = 3.0            # idem tras detectar levantado (rebote de bumpers)


@dataclass
class BladeInputs:
    """None = dato inexistente o viejo (se trata como falla)."""
    request_cutter: bool = False
    request_trimmer: bool = False
    inside: Optional[bool] = None
    grass: Optional[bool] = None
    passes: Optional[int] = None                 # pasadas de la celda actual; <0 o None = desconocida
    tilt_deg: Optional[float] = None
    bumpers: Dict[str, Optional[bool]] = field(default_factory=dict)
    estop: bool = False
    force_cut: bool = False                      # anula solo la condicion de pasto


@dataclass
class BladeDecision:
    cutter: float
    trimmer: float
    reasons: Dict[str, List[str]]               # por que esta apagado cada motor (lista vacia si esta encendido)

    def status(self) -> str:
        def one(name, duty):
            r = self.reasons[name]
            return f'{name}=ON duty={duty:.2f}' if not r else f'{name}=OFF({",".join(r)})'
        return ' | '.join((one('cutter', self.cutter), one('trimmer', self.trimmer)))


def effective_max_duty(max_duty: float, nominal_voltage: float, pack_voltage: float) -> float:
    """Tope real: el configurado, limitado por nominal/pack si ambas tensiones se conocen. Siempre en [0, 1]."""
    limit = min(max(max_duty, 0.0), 1.0)
    if nominal_voltage > 0.0 and pack_voltage > 0.0:
        limit = min(limit, nominal_voltage / pack_voltage)
    return limit


def tilt_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Inclinacion [grados] del eje z del robot respecto de la vertical (roll y pitch combinados)."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return math.nan
    x, y = x / norm, y / norm
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


class BladeCore:
    def __init__(self, params: BladeParams):
        self.p = params
        self.lifted_until = -math.inf
        self.tipped_until = -math.inf

    def evaluate(self, inp: BladeInputs, now: float) -> BladeDecision:
        p = self.p

        # ── Condiciones comunes a los dos motores ──
        common: List[str] = []
        if inp.estop:
            common.append('estop')
        if inp.inside is None:
            common.append('geofence_sin_dato')
        elif not inp.inside:
            common.append('fuera_del_cerco')

        bumpers = {n: inp.bumpers.get(n) for n in BUMPERS}
        if any(v is None for v in bumpers.values()):
            common.append('bumpers_sin_dato')           # sin ellos no se puede detectar el levantado
        if any(bumpers[a] and bumpers[b] for a, b in LIFT_PAIRS):
            self.lifted_until = now + p.lift_hold_s
        if now < self.lifted_until:
            common.append('levantado')

        if inp.tilt_deg is None or math.isnan(inp.tilt_deg):
            common.append('imu_sin_dato')
        elif inp.tilt_deg > p.max_tilt_deg:
            self.tipped_until = now + p.tilt_hold_s
        if now < self.tipped_until:
            common.append('volcado')

        if inp.passes is None or inp.passes < 0:
            common.append('pasadas_desconocidas')
        elif inp.passes >= p.max_passes:
            common.append('celda_con_demasiadas_pasadas')

        reasons = {'cutter': list(common), 'trimmer': list(common)}
        if not inp.request_cutter:
            reasons['cutter'].append('sin_pedido')
        if not inp.request_trimmer:
            reasons['trimmer'].append('sin_pedido')
        if p.require_grass and not inp.force_cut:        # solo la cuchilla; force_cut anula solo esto
            if inp.grass is None:
                reasons['cutter'].append('pasto_sin_dato')
            elif not inp.grass:
                reasons['cutter'].append('sin_pasto')

        cutter = effective_max_duty(p.max_duty_cutter, p.cutter_nominal_voltage, p.pack_voltage)
        trimmer = effective_max_duty(p.max_duty_trimmer, p.trimmer_nominal_voltage, p.pack_voltage)
        return BladeDecision(0.0 if reasons['cutter'] else cutter,
                             0.0 if reasons['trimmer'] else trimmer, reasons)
