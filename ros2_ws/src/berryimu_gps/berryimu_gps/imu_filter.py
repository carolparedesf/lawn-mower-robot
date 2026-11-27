"""
Ganancias del filtro complementario del IMU, independientes de la frecuencia (sin ROS ni I2C).

accel_gain y mag_gain son la correccion por muestra a gain_ref_hz (se ajustaron a 50 Hz). Con otro
periodo se usa la ganancia equivalente 1 - (1 - g) ** (dt * gain_ref_hz): la correccion por
segundo, y por lo tanto la constante de tiempo del filtro, no cambia al bajar publish_rate_hz.
"""
GAIN_REF_HZ = 50.0


def rate_scaled_gain(gain, dt, ref_hz=GAIN_REF_HZ):
    """Ganancia para un paso de dt [s], equivalente a `gain` por muestra a ref_hz."""
    if dt <= 0.0 or gain <= 0.0:
        return 0.0
    if gain >= 1.0:
        return 1.0
    return 1.0 - (1.0 - gain) ** (dt * ref_hz)
