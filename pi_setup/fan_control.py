#!/usr/bin/env python3
"""Control del ventilador de la Raspberry Pi 4 por temperatura (ON/OFF con histeresis).

Hardware: GPIO18 -> resistencia 1 kohm -> base de un 2N2222 que conmuta el ventilador.
Usa el daemon pigpiod (el mismo que usa cutting_motors_node.py).

Uso:
  python3 fan_control.py                      # enciende a 60 C, apaga a 50 C
  python3 fan_control.py --on 45 --off 40     # umbrales bajos para probar
"""
import argparse
import signal
import sys
import time

import pigpio

FAN_GPIO = 18
TEMP_FILE = '/sys/class/thermal/thermal_zone0/temp'  # miligrados Celsius


def read_temp_c(path=TEMP_FILE):
    with open(path) as f:
        return int(f.read().strip()) / 1000.0


def next_state(temp_c, fan_on, t_on, t_off):
    """Histeresis: enciende al llegar a t_on, apaga al bajar a t_off, y en medio mantiene."""
    if temp_c >= t_on:
        return True
    if temp_c <= t_off:
        return False
    return fan_on


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--on', type=float, default=50.0, help='temperatura de encendido [C]')
    ap.add_argument('--off', type=float, default=40.0, help='temperatura de apagado [C]')
    ap.add_argument('--period', type=float, default=2.0, help='periodo de lectura [s]')
    args = ap.parse_args()
    if args.off >= args.on:
        sys.exit('--off debe ser menor que --on')

    pi = pigpio.pi()
    if not pi.connected:
        sys.exit('pigpiod no esta corriendo (sudo systemctl start pigpiod)')
    pi.set_mode(FAN_GPIO, pigpio.OUTPUT)

    # Que systemd (SIGTERM) tambien pase por el bloque finally
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    fan_on = None
    last_log = 0.0
    try:
        while True:
            try:
                temp = read_temp_c()
                new_state = next_state(temp, bool(fan_on), args.on, args.off)
            except (OSError, ValueError) as e:
                # Sin lectura de temperatura: ante la duda, ventilador encendido
                print(f'error leyendo temperatura ({e}): ventilador ON', flush=True)
                temp, new_state = float('nan'), True

            if new_state != fan_on:
                pi.write(FAN_GPIO, 1 if new_state else 0)
                fan_on = new_state
                print(f'{temp:.1f} C -> ventilador {"ON" if fan_on else "OFF"}', flush=True)

            now = time.monotonic()
            if now - last_log >= 60.0:
                print(f'{temp:.1f} C, ventilador {"ON" if fan_on else "OFF"}', flush=True)
                last_log = now

            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        # A prueba de fallos: si el script termina, deja el ventilador encendido
        pi.write(FAN_GPIO, 1)
        pi.stop()


if __name__ == '__main__':
    main()