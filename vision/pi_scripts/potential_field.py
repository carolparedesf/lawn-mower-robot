import math
import numpy as np

# --- Parámetros a ajustar con el robot real ---
K_ATT = 1.0
RHO0 = 1.5          # distancia de influencia de cada obstáculo (m)
F_MAX = 2.0
GOAL_DIST = 3.0     # la meta se pone 3 m al frente (hasta que navegación dé una real)
FOV_H = 62.0        # campo de visión horizontal de tu cámara, en grados (revisá el dato de tu cámara)
K_REP = {"arbol": 0.8, "barrera": 0.8, "planta": 0.5, "otro": 0.4}

# --- Calibración distancia por visión ---
# Poné un objeto a 0.5 m y a 1.5 m del robot y anotá el valor de y_base (en píxeles) que imprime el script.
CALIB_Y = [300, 420]      # y_base medido (de lejos a cerca; deben ir en orden creciente)
CALIB_D = [1.5, 0.5]      # distancia real en metros para esos y_base

def dist_from_y(y_base):
    """Distancia estimada (m) según qué tan abajo está la base del objeto en la imagen."""
    return float(np.interp(y_base, CALIB_Y, CALIB_D))

def bearing_from_x(x_base, width):
    """Ángulo (rad) respecto al frente. Positivo = a la derecha."""
    return math.radians((x_base / width - 0.5) * FOV_H)

def compute_command(obstacles, v_max=0.4, w_max=1.0, k_w=1.5):
    """
    obstacles: lista de (theta_rad, rho_m, clase).
    Marco del robot: x = adelante, y = izquierda.
    Devuelve (v, w): velocidad lineal (m/s) y angular (rad/s, positivo = girar a la izquierda).
    """
    q = np.array([0.0, 0.0])
    goal = np.array([GOAL_DIST, 0.0])

    f = -K_ATT * (q - goal)
    n = np.linalg.norm(f)
    if n > F_MAX:
        f = f / n * F_MAX

    for theta, rho, clase in obstacles:
        if rho > RHO0 or rho < 0.05:
            continue
        obs = np.array([rho * math.cos(theta), -rho * math.sin(theta)])  # y positivo = izquierda
        d = q - obs
        r = np.linalg.norm(d)
        mag = K_REP.get(clase, 0.4) * (1 / r - 1 / RHO0) / r**2
        f = f + mag * d / r

    n = np.linalg.norm(f)
    if n > F_MAX:
        f = f / n * F_MAX

    heading_err = math.atan2(f[1], f[0])
    w = max(-w_max, min(w_max, k_w * heading_err))
    v = v_max * max(0.0, math.cos(heading_err)) * min(1.0, n / F_MAX + 0.2)
    return v, w
