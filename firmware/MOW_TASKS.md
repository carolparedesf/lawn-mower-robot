# Firmware en tareas (entorno `mow_tasks`)

`mow` (src/main.cpp) es el respaldo y no cambia. `mow_tasks` (src/main_tasks.cpp)
usa las mismas librerias, topics, tipos, QoS, nodo, dominio 42 y 921600.

| Tarea    | Nucleo | Prio | Frecuencia | Hace |
|----------|--------|------|------------|------|
| control  | 1      | 5    | 50 Hz (vTaskDelayUntil) | encoders, PID, PWM, odometria |
| microros | 0      | 3    | continuo   | unica que llama a rcl/rclc; publica, recibe, reconexion |
| sensors  | 1      | 2    | 20 Hz      | sonares, bumpers, levantado, INA228 (500 ms), OLED (1 Hz), buzzer |

Datos compartidos en `lib/shared_state` (un escritor por dato, copias en bloque
con portMUX). Logica pura en `lib/task_logic`, testeada en la PC:

    g++ -std=c++17 -Wall -Wextra -I lib/task_logic test/host/test_task_logic.cpp -o /tmp/t && /tmp/t

## Cargar

    pio run -e mow_tasks -t upload     # tareas
    pio run -e mow -t upload           # volver al respaldo

La primera compilacion de `mow_tasks` reconstruye micro-ROS (~2 min) porque usa
`colcon_tasks.meta` (14 publishers / 6 suscripciones; `mow` sigue con 10 / 5).

## Topics nuevos (solo mow_tasks)

- `robot/events` (std_msgs/String, entrada): lifted, cutting_started,
  navigation_started, perimeter_started, low_battery. Un sonido corto por evento;
  el mismo evento no se repite en menos de 2 s (tampoco si el ESP32 ya lo
  detecto). Textos de mas de 31 caracteres se descartan.
- `esp32/control_period_ms` y `esp32/control_period_max_ms` (std_msgs/Float32,
  1 Hz): periodo real medio y maximo de la tarea de control en el ultimo segundo.

Cambio de frecuencia: `battery/state` pasa de 2 Hz a 1 Hz.

## Seguridad

- PWM 0 e integral 0 si: orden de mas de 0.5 s, sin agente, `estop_external`
  (/estop true), `estop_lifted` (levantado, enganchado) o bateria baja MEDIDA
  (< 15 V, vuelve en 16 V). Sin dato del INA228 los motores funcionan.
- `/estop false` limpia los dos e-stop; si los bumpers de levantado siguen
  apretados, se vuelve a enganchar. Al caer el agente se limpia solo
  `estop_external`.
- Desconexion tras 3 pings fallidos seguidos (cada 200 ms, timeout 100 ms).
