# Navegación: modos, multiplexor y safety gate

El bridge ya no navega: solo traduce MQTT ↔ ROS. La navegación y la seguridad viven en el
paquete `lawnbot_nav` (`ros2_ws/src/lawnbot_nav`). El contrato MQTT con la app no cambia.

```
App ──MQTT──► bridge ──► /robot_mode (heartbeat 1 Hz) ──► mode_manager ──► /nav/active_mode
                │                                                              │
                ├──► /cmd_vel_manual ─┐              stubs: /cmd_vel_parallel|random|perimeter (20 Hz)
                └──► /geofence_raw    └─► twist_mux ─► /cmd_vel_mux ─► safety_gate ─► /cmd_vel ─► ESP32
                                                       sonar/* + bumper/* ─────────────┘
bridge ◄── /nav/active_mode, /nav/safety_alert, /battery/state   (→ robot/status cada 1 s)
```

## Piezas

| Nodo | Qué hace |
|---|---|
| `bridge/bridge.py` | MQTT ↔ ROS. `robot/status` lleva el modo real (`/nav/active_mode`), la batería real y la alerta |
| `mode_manager` | `/robot_mode` → `/nav/active_mode`. Modo desconocido o sin heartbeat por 3 s → `idle` |
| `nav_stub` ×3 | Patrones a ciegas (paralelo, aleatorio, perímetro) a 20 Hz mientras su modo está activo |
| `twist_mux` | Prioridad: manual > perimeter > parallel > random, timeouts 0,5 s. Salida `/cmd_vel_mux` |
| `safety_gate` | Filtra `/cmd_vel_mux` y publica `/cmd_vel` a 20 Hz. Solo limita o anula |

Parámetros: `ros2_ws/src/lawnbot_nav/config/*.yaml` (umbrales, timeouts, sonares habilitados, bumpers).

### Reglas del safety_gate

| Condición | Efecto |
|---|---|
| Bumper frontal (B1 o B2) | prohíbe avance; retroceso y giro pasan |
| Bumper trasero (B5) | prohíbe retroceso |
| Sonar: mínimo de los habilitados < 0,30 m | avance proporcional |
| Sonar: mínimo < 0,15 m | avance prohibido |
| Sonar habilitado o bumper frontal sin datos > 1,0 s | avance prohibido (retroceso y giro permitidos) |
| B3 / B4 | sin efecto (solo se registran en el log) |

Alertas (`/nav/safety_alert` → campo `alert` de `robot/status`): `none`, `sensor_timeout`,
`bumper_front`, `bumper_back`, `obstacle_stop`, `obstacle_near`.

## Instalación (en la Pi)

```bash
sudo apt install ros-humble-twist-mux
cd ~/ws_new && colcon build --packages-select lawnbot_nav
```

## Lanzar

```bash
ros2 launch lawnbot_nav nav.launch.py      # mode_manager + stubs + twist_mux + safety_gate
# el bridge corre aparte (bridge.service)
```

**Teleop:** debe ir por la entrada manual, nunca directo a `/cmd_vel`:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_manual
```

**`/cmd_vel` debe tener un solo publicador (`safety_gate`):**

```bash
ros2 topic info /cmd_vel -v
```

## Plan de pruebas (robot levantado primero, cuchillas sin conectar)

Atención: el firmware entra en e-stop (enganchado) si se presionan a la vez B3+B4, B1+B5 o B2+B5.
Al probar bumpers, apretar uno solo. Para liberar: `ros2 topic pub --once /estop std_msgs/msg/Bool "{data: false}"`.

**1. En seco, sin agente.** `ros2 launch lawnbot_nav nav.launch.py`. Sin errores; `ros2 node list` muestra
`mode_manager`, `nav_stub_*`, `twist_mux`, `safety_gate`. Con `ros2 topic echo /cmd_vel` debe haber ceros a 20 Hz.

**2. Entradas a mano** (con `ros2 topic echo /cmd_vel_mux` y `/cmd_vel` abiertos). Los sensores hacen falta
(paso 4), si no el avance sale bloqueado.
```bash
ros2 topic pub -r 20 /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
```
Al cortar con Ctrl+C, ambas salidas vuelven a 0 en ~0,5 s. Repetir con `/cmd_vel_perimeter`,
`/cmd_vel_parallel`, `/cmd_vel_random`; publicar dos a la vez: gana la de mayor prioridad.

**3. Modos y heartbeat** (el publicador debe ser `transient_local`):
```bash
ros2 topic pub -r 1 /robot_mode std_msgs/msg/String "{data: 'auto_parallel'}" \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /nav/active_mode --qos-durability transient_local --qos-reliability reliable
ros2 topic hz /cmd_vel_parallel     # ~20 Hz solo mientras el modo está activo
```
Cambiar a `auto_random`, `auto_perimeter`, `manual`, `stop`. Un modo inventado → `idle`.
Cortar el publicador: a los 3 s `/nav/active_mode` pasa a `idle`.

**4. Sensores simulados** (sin tocar el robot):
```bash
for t in left center right; do
  ros2 topic pub -r 10 /sonar/$t sensor_msgs/msg/Range "{range: 1.5, min_range: 0.02, max_range: 1.5}" >/dev/null &
done
ros2 topic pub -r 10 /bumper/B1_front_left  std_msgs/msg/Bool "{data: false}" >/dev/null &
ros2 topic pub -r 10 /bumper/B2_front_right std_msgs/msg/Bool "{data: false}" >/dev/null &
```
Con avance publicado en `/cmd_vel_manual`: `/cmd_vel` pasa igual. Luego reemplazar un sonar por `range: 0.225`
(esperado: avance a la mitad), `0.10` (avance 0, giro y retroceso intactos), `B1` en `true` (avance 0),
`B5` en `true` con retroceso (retroceso 0). Cortar un sensor: a 1 s el avance se bloquea (`sensor_timeout`).

**5. Con agente y sensores reales, robot levantado.** Repetir el paso 4 con los sensores del robot: un
bumper por vez, la mano frente al sonar. Comprobar que las ruedas obedecen lo que sale en `/cmd_vel`.

**6. Fallas.** Matar `safety_gate`: las ruedas se frenan por el watchdog del firmware (500 ms). Matar el
bridge en modo automático: a los 3 s el modo pasa a `idle`. Desconectar el celular.

**7. En el suelo,** velocidad baja y el e-stop listo.

## Seguidor de waypoints (`waypoint_follower`)

Pure pursuit sobre una lista de waypoints. **No arranca solo**: espera `true` en `/follower/enable`.
Publica en **`/cmd_vel_waypoint`** (entrada de `twist_mux`, prioridad 50), así la orden pasa por el
`safety_gate` y `/cmd_vel` conserva un único publicador. Config: `config/waypoint_follower.yaml`
(cuadrado de 3 m × 3 m en `odom`, `relative_to_start: true`).

```bash
ros2 launch lawnbot_nav nav.launch.py                 # mux + gate + modos
ros2 launch lawnbot_nav waypoint_follower.launch.py   # el seguidor (queda en IDLE)
ros2 topic pub --once /follower/enable std_msgs/msg/Bool "{data: true}"    # iniciar / reanudar
ros2 topic pub --once /follower/enable std_msgs/msg/Bool "{data: false}"   # pausar
ros2 topic echo /follower/state                       # IDLE | RUNNING | PAUSED | DONE | ERROR
```

**Estados y reglas**
- `enable true` en `RUNNING` es idempotente (no reinicia). Tras `DONE` o `ERROR`, un `true` reinicia desde el primer waypoint.
- Se pausa solo (`PAUSED`) si: llega **cualquier** mensaje a `/cmd_vel_manual` (incluido un cero; el botón *stop* de la app lo
  genera), `/estop` es `true`, o el modo activo sale de `allowed_modes` (por defecto solo `manual`: la app debe estar en modo manual).
  Tras esas pausas hay que publicar `false` y luego `true` para reanudar. Con `/estop` activo, `enable` se rechaza.
- Sin pose (TF caída o más vieja que 0,5 s) publica cero; a los 3 s pasa a `ERROR`.
- `ERROR no_progress`: 20 s sin acercarse al waypoint. Al girar en el lugar, reducir el error de rumbo cuenta como avance.
- Al terminar, pausar, dar error o apagar el nodo publica cero.
- `relative_to_start: true`: (0, 0) es la pose del robot al habilitar y +x su rumbo en ese momento. En `odom`, el origen del EKF
  es la pose donde arrancó, no donde está el robot ahora. Con `frame_id: map` usar `relative_to_start: false` y waypoints absolutos.
- Los waypoints son una lista **plana** `[x0, y0, x1, y1, ...]` con decimales.

**Zona muerta del PWM:** al girar en el lugar la rueda va a `w · 0,1775` m/s (a 0,6 rad/s: 0,1065 m/s). Si la zona muerta de tu motor
es mayor, no gira: o subís `max_angular` por encima de `zona_muerta / 0,1775` o el seguidor termina en `ERROR no_progress`.
`min_turn` y `min_linear` fijan pisos de velocidad para eso.

### Prueba en seco (sin robot)

0. **Sin ROS:** `cd ros2_ws/src/lawnbot_nav && PYTHONPATH=. python3 test/test_follower.py` (y `test_logic.py`).
1. **Robot simulado.** No correr `ekf_local` ni la odometría real a la vez (habría dos fuentes de `odom → base_footprint`).
   ```bash
   ros2 launch lawnbot_nav follower_sim.launch.py x:=2.0 y:=1.0 yaw:=0.5    # fake_robot + seguidor
   ros2 topic echo /follower/state
   ros2 topic pub --once /follower/enable std_msgs/msg/Bool "{data: true}"
   ```
   Debe cerrar el cuadrado en ~67 s y terminar en `DONE` a menos de 0,2 m del inicio. Ver el movimiento con
   `ros2 run tf2_ros tf2_echo odom base_footprint`. Probar además: `enable true` repetido (no reinicia); `false` y `true`
   (pausa y reanuda); `ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/Twist "{}"` (pausa; exige `false` + `true`);
   `/estop true`; `wheel_deadzone:=0.08` (sin pisos debe dar `ERROR no_progress`).
2. **Pérdida de TF:** correr `ros2 run lawnbot_nav fake_robot` y el seguidor por separado, y cortar `fake_robot`: a los 0,5 s
   `/cmd_vel_waypoint` queda en cero y a los 3 s el estado pasa a `ERROR`.
3. **Pose fija** (para ver el signo de las órdenes): `ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --yaw 1.57
   --frame-id odom --child-frame-id base_footprint` y el seguidor con `-p require_active_mode:=false -p relative_to_start:=false
   -p waypoints:="[3.0, 0.0]"`. Con la pose estática el robot no se mueve; se miran las órdenes:

   | Pose falsa (x, y, yaw) | Objetivo | Orden esperada |
   |---|---|---|
   | (0, 0, 0) | (3, 0) | `v = 0,2`, `w ≈ 0` |
   | (0, 0, +1,57) | (3, 0) | `v = 0`, `w < 0` (gira a la derecha) |
   | (0, 0, −1,57) | (3, 0) | `v = 0`, `w > 0` |
   | (0, −0,5, 0) | (3, 0) | `v > 0`, `w > 0` (corrige hacia la izquierda; segmento desde la pose inicial, no desde el origen) |
   | (2,9, 0, 0) | (3, 0) | pasa a `DONE` |

4. **Con la cadena real** (`nav.launch.py` + `follower_sim.launch.py cmd_topic:=cmd_vel` + sensores simulados): con un sonar en 0,20 m el
   avance se frena y el seguidor termina en `ERROR no_progress` a los 20 s.
5. **Robot levantado**, luego en el suelo a 0,1 m/s y después 0,2 m/s, con el e-stop listo.

## Pendientes

- **App Flutter:** registrar un MQTT Last Will en `robot/cmd/mode` con `{"mode":"idle"}`, para que si el
  celular se desconecta el robot pase a `idle`.
- **Firmware (no tocado):** bumpers y sonares salen a 5 Hz (`timer_sensors`), hasta ~200 ms de latencia.
  Una mejora futura es publicar los bumpers por evento.
- **Frecuencia de `robot/cmd/move` en la app** (medida leyendo el código): el D-pad repite cada 150 ms
  (≈ 6,7 Hz) mientras está presionado. El joystick llama a su `listener` cada 100 ms (`period` por defecto de
  `flutter_joystick` 0.1.0, también con el stick quieto) y la app publica con un retardo de 50 ms: ≈ 10 Hz.
  Ambos quedan bien por debajo de los 0,5 s de `twist_mux`, `safety_gate` y del watchdog del firmware.
- **Despliegue:** copiar el `bridge.py` nuevo a la Pi (`~/pi/bridge.py`) con backup, y reiniciar `bridge.service`.
  Hasta entonces la Pi sigue con el bridge viejo.
