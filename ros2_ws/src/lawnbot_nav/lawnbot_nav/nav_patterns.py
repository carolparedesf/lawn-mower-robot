"""Patrones de los stubs de navegacion (sin dependencias de ROS).

Cada patron entrega (linear_x, angular_z) segun el tiempo transcurrido desde que se activo.
Son los mismos movimientos a ciegas que tenia el bridge, pero evaluados de forma continua.
"""
import random


class ParallelPattern:
    """Avanza, gira, avanza, gira al lado contrario (3 s / 1 s / 3 s / 1 s)."""

    def __init__(self, linear=0.3, turn=1.0, straight_s=3.0, turn_s=1.0):
        self.phases = [
            (linear, 0.0, straight_s),
            (0.0, turn, turn_s),
            (linear, 0.0, straight_s),
            (0.0, -turn, turn_s),
        ]
        self.cycle = sum(p[2] for p in self.phases)

    def command(self, elapsed):
        t = elapsed % self.cycle
        for v, w, duration in self.phases:
            if t < duration:
                return v, w
            t -= duration
        return 0.0, 0.0


class RandomPattern:
    """Elige al azar entre avanzar y girar a cada lado, cada 1 a 3 s."""

    def __init__(self, linear=0.3, turn=1.0, min_s=1.0, max_s=3.0, rng=None):
        self.moves = [(linear, 0.0), (0.0, turn), (0.0, -turn)]
        self.min_s = min_s
        self.max_s = max_s
        self.rng = rng or random.Random()
        self.current = (0.0, 0.0)
        self.until = 0.0

    def command(self, elapsed):
        if elapsed >= self.until:
            self.current = self.rng.choice(self.moves)
            self.until = elapsed + self.rng.uniform(self.min_s, self.max_s)
        return self.current


class PerimeterPattern:
    """Avance recto constante."""

    def __init__(self, linear=0.2):
        self.linear = linear

    def command(self, elapsed):
        return self.linear, 0.0


def make_pattern(name, params):
    if name == 'parallel':
        return ParallelPattern(**params)
    if name == 'random':
        return RandomPattern(**params)
    if name == 'perimeter':
        return PerimeterPattern(**params)
    raise ValueError(f'patron desconocido: {name}')
