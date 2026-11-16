"""Geometria del chasis (plano CAD) y montaje de los sonares, compartida por el simulador y los nodos. Sin ROS.

Medidas del plano CAD, en metros. El origen es base_footprint; el frente del cuerpo queda en x = BODY_FRONT_X (la misma
referencia que el URDF y los montajes de sensores de abajo).

Huella: un bloque delantero ancho (400 x 226 mm) con las esquinas delanteras redondeadas (radio 60 mm) y un bloque trasero
mas angosto (250 mm de ancho; el largo total es 436 mm, o sea 210 mm de bloque trasero).

Angulo de los sonares de las esquinas: UN solo parametro de ROS, `corner_sonar_angle_deg` (config/robot_geometry.yaml), que leen
fake_robot, potential_field, coverage_map y perimeter, y de donde sale el yaw de los sonares del URDF (mow_simplified.urdf.xacro).
Es el angulo respecto del frente (0 = mira al frente, 90 = de costado), positivo hacia afuera: izquierdo +angulo, derecho -angulo.
PROVISORIO (30 grados) hasta medirlo en el robot.
"""
from typing import Dict, Tuple

CORNER_SONAR_ANGLE_DEG = 30.0

# Montaje (x, y) de los sonares respecto de base_footprint: uno al centro del frente y uno en cada esquina delantera
SONAR_MOUNTS = {'left': (0.194, 0.136), 'center': (0.198, 0.0), 'right': (0.193, -0.1435)}

# Huella del chasis (plano CAD)
BODY_FRONT_X = 0.2137           # cara frontal, en el marco de base_footprint
BODY_LENGTH = 0.436             # largo total
BODY_FRONT_DEPTH = 0.226        # profundidad del bloque delantero (ancho)
BODY_FRONT_WIDTH = 0.400
BODY_REAR_WIDTH = 0.250
CORNER_RADIUS = 0.060           # esquinas delanteras


def sonar_yaws(corner_angle_deg: float) -> Dict[str, float]:
    """Yaw de montaje [grados] de cada sonar a partir del angulo de las esquinas."""
    return {'left': float(corner_angle_deg), 'center': 0.0, 'right': -float(corner_angle_deg)}


def sonar_mount(name: str, corner_angle_deg: float) -> Tuple[float, float, float]:
    """(x, y, yaw en grados) del sonar `name`."""
    x, y = SONAR_MOUNTS[name]
    return x, y, sonar_yaws(corner_angle_deg)[name]
