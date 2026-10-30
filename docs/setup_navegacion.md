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
| `safety_gate` | Filtra `/cmd_vel_mux` y publica `/cmd_vel` a 20 Hz. Solo limita o anula. Fusiona las alertas de `/nav/alert_in` |
| `geofence_node` | Cerco virtual: `/geofence_raw` (lat/lon) → polígono en `map`, `/geofence/inside`, `/geofence/distance_to_edge` |

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

## Bloque A: cerco virtual, rutas por topic y auto-inicio

**Rama `feat/nav-autonoma`.** Todo movimiento sale a su propio `/cmd_vel_<algoritmo>`, pasa por `twist_mux` y `safety_gate`, y nadie
publica en `/cmd_vel` salvo el gate.

### geofence_node

Recibe `/geofence_raw` (el JSON de la app: `{"points":[{"lat":..,"lng":..}, ...]}`), lo convierte a metros del frame `map` y publica:

| Topic | Tipo | Contenido |
|---|---|---|
| `/geofence/polygon` | `PolygonStamped` (latched) | polígono en `map` para RViz; vacío si no hay cerco válido |
| `/geofence/inside` | `Bool` (10 Hz) | dentro del cerco, con histéresis (`hysteresis_in` = 5 lecturas para pasar a `true`, `hysteresis_out` = 2 para pasar a `false`) |
| `/geofence/distance_to_edge` | `Float32` | distancia al borde: **positiva dentro, negativa fuera, `NaN` si se desconoce** |
| `/geofence/status` | `String` (latched) | `NO_POLYGON`, `INVALID:<motivo>`, `WAITING_CONVERSION`, `NO_POSE`, `OK` |

- **Validación:** mínimo 4 vértices (si el último repite al primero se descarta), aristas de al menos 0,5 m, área de al menos 2 m²
  y polígono simple (sin autointersecciones ni vértices que rocen otra arista). Se normaliza a sentido antihorario. La app deja enviar
  3 puntos; el nodo los rechaza (`INVALID:menos de 4 vertices`).
- **Falla segura:** sin cerco, sin pose (TF ausente o de más de 0,5 s) o con el cerco sin reconvertir, `inside` es `false` y la
  distancia `NaN`. Un valor desconocido apaga `inside` de inmediato, sin esperar la histéresis.
- **Conversión:** con el servicio **`/fromLL` de `navsat_transform`** (`robot_localization/srv/FromLL`), es decir, con la misma
  transformación que usa el filtro. El cerco se guarda siempre en lat/lon (también en `~/.ros/lawnbot_geofence.json`, así sobrevive
  a reinicios) y se reconvierte cada vez que el servicio vuelve a estar disponible y cada `reconvert_period_s`, porque el origen de
  `map` cambia entre arranques. Mientras el servicio está caído el cerco no tiene validez.
  `navsat_transform` responde `(0, 0)` sin error si su transformación no está lista: el nodo lo detecta y reintenta.
- `conversion_mode: datum` (con `datum_lat` y `datum_lng`) usa un plano tangente local y es solo para pruebas sin GPS. **No** es un
  respaldo automático: mezclaría marcos distintos.
- No mueve el robot, así que corre en cualquier modo (excepción a "un nodo no hace nada hasta que su modo está activo").
- **Precisión:** con el error típico de un GPS común (2–3 m), `inside` y la distancia no son más exactos que eso.
  El margen de seguridad frente al borde tiene que ser mayor que ese error.

**Cómo probarlo**
1. Sin ROS: `cd ros2_ws/src/lawnbot_nav && PYTHONPATH=. python3 test/test_geofence.py`.
2. Nodos sin ROS (capa de ROS falsa, solo en una máquina **sin** ROS 2): `python3 tools/node_scenarios.py`.
3. Con ROS y `fake_robot` (sin GPS): `ros2 run lawnbot_nav geofence_node --ros-args -p conversion_mode:=datum -p datum_lat:=-25.3 -p datum_lng:=-57.6`
   y `ros2 launch lawnbot_nav follower_sim.launch.py` con `frame_id:=map`. Publicar un cerco cuadrado de ~20 m alrededor del datum:
   ```bash
   ros2 topic pub --once /geofence_raw std_msgs/msg/String "{data: '{\"points\":[{\"lat\":-25.3,\"lng\":-57.6},{\"lat\":-25.3,\"lng\":-57.5998},{\"lat\":-25.2998,\"lng\":-57.5998},{\"lat\":-25.2998,\"lng\":-57.6}]}'}" \
     --qos-durability transient_local --qos-reliability reliable
   ros2 topic echo /geofence/status --qos-durability transient_local --qos-reliability reliable
   ros2 topic echo /geofence/inside
   ```
   Con `fake_robot` en `x:=10 y:=11` debe dar `OK` e `inside: true`; moviéndolo fuera, `false` a las 2 lecturas.
4. **En la Pi, con GPS:** comprobar que el servicio exista: `ros2 service list | grep -i ll` (deben aparecer `/fromLL` y `/toLL`) y
   `ros2 service type /fromLL` (`robot_localization/srv/FromLL`). El servicio solo responde con fix GPS y `ekf_global` activos
   (`map → odom` publicado); antes de eso el estado es `WAITING_CONVERSION`.

### waypoint_follower: rutas por topic y auto-inicio

- **`/follower/path`** (`nav_msgs/Path`, latched): se acepta si `header.frame_id` coincide con el `frame_id` del seguidor, sin NaN y
  hasta `max_path_points`. Las rutas por topic son **absolutas** (`relative_to_start` vale solo para el YAML). Una ruta nueva se
  rechaza con el seguidor en `RUNNING`; en cualquier otro estado reemplaza a la anterior y deja el estado en `IDLE`. Un `Path` vacío
  borra la ruta (y si estaba navegando, publica cero).
- **`allowed_modes: [manual, auto_parallel]`.**
- **Auto-inicio** (`auto_start_modes: [auto_parallel]`): al *entrar* a ese modo (no al arrancar el nodo ya dentro de él), con una ruta
  cargada, arranca solo **únicamente si `/geofence/status` es `OK` y `/geofence/inside` es `true`**. Si no, no arranca y avisa por
  `/nav/alert_in` (`follower:geofence_not_ready`) cada segundo; a los `auto_start_timeout_s` (30 s) desiste (`auto_start_timeout`).
  Elegir el modo de nuevo lo rearma. Si el seguidor pasa a `ERROR`, avisa con `follower:tf_lost` o `follower:no_progress`.
- **Tolerancia por ruta:** la tolerancia efectiva es `min(goal_tolerance, 0,4 × tramo más corto)`. Para líneas separadas 0,27 m es
  0,108 m. En simulación el seguidor no saltaba waypoints con 0,2 m, pero cortaba la esquina hasta 0,2–0,3 m antes del final de cada línea;
  con la tolerancia derivada ese corte baja a 0,06–0,16 m. Las rutas del YAML (tramos de varios metros) no cambian.

**Cómo probarlo**
1. Sin ROS: `PYTHONPATH=. python3 test/test_route.py` (rutas, tolerancia, auto-inicio, alertas y recorrido de líneas paralelas con el robot simulado).
2. Con `fake_robot` (ROS): `ros2 launch lawnbot_nav follower_sim.launch.py x:=2.0 y:=2.0` con `frame_id:=map`, y publicar una ruta:
   ```bash
   ros2 topic pub --once /follower/path nav_msgs/msg/Path "{header: {frame_id: map}, poses: [
     {pose: {position: {x: 2.0, y: 2.0}}}, {pose: {position: {x: 8.0, y: 2.0}}},
     {pose: {position: {x: 8.0, y: 2.27}}}, {pose: {position: {x: 2.0, y: 2.27}}}]}" \
     --qos-durability transient_local --qos-reliability reliable
   ```
   Con `require_active_mode:=false` se habilita con `/follower/enable`. Para probar el auto-inicio, con el modo real: publicar
   `/robot_mode` en `auto_parallel` (con `nav.launch.py`) y ver que **no** arranca con `/geofence/status` distinto de `OK` o con
   `/geofence/inside` en `false`, y que avisa en `/nav/alert_in`; con ambos correctos, arranca.

### Alertas desde otros nodos

Los nodos publican en **`/nav/alert_in`** con formato `fuente:alerta` (p. ej. `follower:geofence_not_ready`; `follower:none` la limpia).
El `safety_gate` es el único que publica `/nav/safety_alert`: toma la de mayor prioridad entre las suyas y las vigentes de
`/nav/alert_in` (vencen a los `alert_ttl` = 3 s sin renovarse). Prioridad: `sensor_timeout`, `bumper_front`, `bumper_back`,
`obstacle_stop`, `estop`, `geofence_breach`, `tf_lost`, `no_progress`, `geofence_not_ready`, `no_pose`, `auto_start_timeout`, `obstacle_near`.
El bridge la manda como `alert` en `robot/status`.

## Pendientes

- **App Flutter:** exigir 4 puntos para enviar un cerco (hoy permite 3 y el nodo los rechaza).
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
