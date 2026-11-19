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
| Sonar: mínimo ≤ 0,10 m (`sonar_stop_distance`) | avance prohibido; **se libera recién por encima de 0,15 m** (histéresis `sonar_stop_hysteresis`, 0,05) |
| Sonar: el obstáculo se aleja | la velocidad permitida **sube con rampa** (`sonar_release_accel`, 0,5 m/s²) y **baja de inmediato** |
| Sonar habilitado o bumper frontal sin datos > 1,0 s | avance prohibido (retroceso y giro permitidos) |
| B3 / B4 | sin efecto (solo se registran en el log) |

**Rampa e histéresis (contra el temblor):** los sonares saltan y la velocidad dependía de la distancia al instante, así que cerca de un umbral el robot "temblaba".
- **Histéresis del bloqueo total:** entra a `≤ sonar_stop_distance` (0,10 m en el YAML) y sale a `> stop + sonar_stop_hysteresis` (0,05 → 0,15 m). Cada sonar con su umbral propio tiene su propia banda; un sonar ignorado (laterales en `auto_perimeter`) olvida su bloqueo; una lectura NaN lo conserva.
- **Rampa de subida:** el factor de avance (0–1) multiplica la orden, así que equivale a una velocidad permitida de `factor × max_linear`. Esa velocidad crece como máximo `sonar_release_accel` (0,5 m/s², `0` = sin rampa) y baja de inmediato. Un bumper o un dato viejo también la ponen en 0, y al liberarse vuelve con rampa.
- Sigue valiendo: nunca amplifica ni invierte una velocidad, y el retroceso y el giro no dependen de los sonares.
- **Parada a 0,10 m** (`sonar_stop_distance: 0.10`, `sonar_slow_distance: 0.30`): entre 0,30 y 0,10 m el avance baja de forma proporcional. En simulación con ruido y el filtro, el robot queda a ~9–10 cm de la pared a 0,15 m/s, ~7–8,5 cm a 0,3 m/s y **solo 2–4 cm a 0,5 m/s** (el retraso del sonar y del filtro). Si quieres que nunca pase de ~8 cm, limita `max_linear` a 0,3 en el mismo YAML. El bumper sigue siendo la última barrera.

**Umbrales por sonar e ignorar los laterales en `auto_perimeter`** (`config/safety_gate.yaml`):
- `sonar_<left|center|right>_slow_distance` / `_stop_distance`: umbral propio de cada sonar (`-1` = usar los globales 0,30 / 0,15 m). Gana el sonar más restrictivo.
- `ignore_side_sonars_in_perimeter` (`true`): mientras el modo activo es `auto_perimeter` los sonares **laterales** no frenan ni cuentan para el dato vigente
  (seguir una pared de cerca los dispararía). **El sonar central y los bumpers siguen protegiendo siempre** (un sonar central sin datos sigue bloqueando el
  avance). Si el modo deja de ser vigente (`mode_timeout`, 3 s) o es otro, se vuelven a considerar todos. El gate solo limita o anula: nunca amplifica una velocidad.

Alertas (`/nav/safety_alert` → campo `alert` de `robot/status`): `none`, `sensor_timeout`,
`bumper_front`, `bumper_back`, `obstacle_stop`, `obstacle_near`.

`bumper_front` sale solo de `bumper_front_topics` (B1, B2) y `bumper_back` solo de `bumper_back_topics` (B5). Los laterales (B3, B4,
`bumper_monitor_topics`) son de monitoreo: no generan ninguna alerta. En el simulador (`sim_world`) ver *Esquinas redondeadas y bumpers
en el simulador* más abajo.

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

## Filtro de sonares (`sonar_filter`)

Nodo `sonar_filter` (`config/sonar_filter.yaml`): lee `sonar/left|center|right` y publica `sonar_filtered/left|center|right` (`sensor_msgs/Range`, mismo header y stamp que la última lectura). Por sonar:
- **Lecturas inválidas** (NaN, inf, ≤ 0, menores que `min_range`): no se mezclan ni se publican. Si un sonar deja de dar datos válidos, los consumidores ven el dato viejo y fallan seguro (`sensor_timeout`: el gate bloquea el avance).
- **`max_range` (sin eco):** no se mezcla con las válidas. Se sigue publicando la mediana vigente y recién tras `max_range_run` (3) seguidas se publica `max_range` (se borra la historia: camino libre).
- **Saltos:** una lectura a más de `max_jump` (0,5 m) de la mediana se rechaza como espuria; si **la siguiente coincide** con ella (dentro de `max_jump`) se acepta como nivel nuevo (un obstáculo real que aparece de golpe se detecta a la segunda lectura). Tras publicar `max_range`, una lectura baja aislada tampoco pasa sin confirmación.
- **Mediana** de las últimas `window` (5) lecturas válidas. Para no frenar tarde, si las dos últimas están más de `approach_margin` (0,03 m) por debajo de la mediana (acercamiento sostenido, no ruido) se publica la mayor de las dos.

**Quién lo usa:** `nav.launch.py use_sonar_filter:=true` (por defecto) lanza el filtro y le pone `sonar_topic_prefix: sonar_filtered` a `safety_gate`, `random_walk`, `perimeter`, `coverage_map` y `potential_field`; con `use_sonar_filter:=false` no se lanza el filtro y todos leen `sonar/*` directo. Si el filtro se cae con `true`, los consumidores quedan sin datos y el gate bloquea el avance. Sin el launch, `sonar_topic_prefix` vale `sonar`.

**Costo medido en simulación** (lecturas ruidosas: σ = 3 cm, 6 % de picos hacia abajo, 8 % de pérdidas de eco, sonares a 5 Hz): la desviación de la velocidad de salida baja a ~½ y los cambios de sentido a ~⅓ (hasta ~1/20 lejos del umbral), pero el filtro **le quita margen al frenar**: hasta ~4 cm a 0,5 m/s y casi nada a 0,15–0,3 m/s (el período de 5 Hz del sonar de por sí ya son 0,1 m a 0,5 m/s). Con una pared a 0,5 m/s la distancia mínima fue 5,5 cm (sin filtro 9,2 cm); a 0,15–0,3 m/s, 10–13 cm. Los bumpers siguen siendo la última barrera; conviene no pasar de ~0,3 m/s en el patio. No es un resultado con el robot real.

Pruebas sin ROS: `PYTHONPATH=. python3 test/test_sonar_filter.py` y `test/test_gate_ramp.py` (histéresis, rampa, "nunca amplifica ni invierte" y lecturas ruidosas simuladas con y sin filtro); escenarios [15]–[17] de `tools/node_scenarios.py`.
En el robot: `ros2 topic echo /sonar/center --field range` frente a `/sonar_filtered/center --field range` con algo cerca: el filtrado debe verse más estable.

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

- **Conversión desde `nav.launch.py`:** argumentos `geofence_conversion_mode` (`navsat` por defecto, o `datum`), `datum_lat` y `datum_lng`
  (0.0 por defecto); se pasan al `geofence_node` **después** de `geofence.yaml`, así que lo pisan. Para probar sin GPS:
  `ros2 launch lawnbot_nav nav.launch.py geofence_conversion_mode:=datum datum_lat:=-25.3 datum_lng:=-57.6`. El nodo **no** aplica
  `conversion_mode` en caliente con `ros2 param set`: hay que lanzarlo con el modo ya elegido. `datum` con `datum_lat`/`datum_lng` en 0.0 es un error y el nodo no arranca.

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
`obstacle_stop`, `estop`, `imu_lost`, `geofence_breach`, `outside_fence`, `tf_lost`, `no_progress`, `perimeter_blocked`, `perimeter_lost`, `perimeter_misconfigured`, `random_blocked`, `geofence_not_ready`, `coverage_error`, `no_pose`, `auto_start_timeout`, `obstacle_near`.
El bridge la manda como `alert` en `robot/status`.

### Vigilante del IMU (`imu_watchdog`, `config/imu_watchdog.yaml`)

Los EKF toman el giro **solo del IMU** (`odom0` de `ekf_local` y `ekf_global` fusiona solo `vx`; ver `lawnbot_localization/config`). Si `/imu/data` se calla, el filtro sigue publicando TF
(`sensor_timeout` 0,2 s solo hace que prediga sin medición) y **nadie se entera**: el rumbo queda solo con la predicción, que no hace decaer la velocidad angular (si el IMU muere girando, el filtro
sigue "girando") y, en `ekf_global`, sin yaw absoluto. Por eso:
- `imu_watchdog` mide la edad de `/imu/data`; pasados **`imu_timeout` (0,5 s, parámetro; ≤ 0 lo apaga)** sin mensajes publica **`imu:imu_lost`** en `/nav/alert_in` (la renueva cada segundo y al volver publica `imu:none`).
  Un IMU que nunca arranca también cuenta (el plazo corre desde que arrancó el vigilante). La lógica está en `safety_logic.imu_is_lost`.
- El `safety_gate` pone a **cero velocidad lineal y angular** mientras haya `imu_lost` vigente (`alert_ttl`, 3 s) **y** el modo vigente sea automático (`auto_parallel`, `auto_random`, `auto_perimeter`;
  `safety_logic.imu_blocks_motion`). **`manual`, `idle` y `stop` no se tocan.** `imu_lost` sale por `/nav/safety_alert` → `robot/status` → la app ("Se perdió el IMU: modos automáticos detenidos"),
  con prioridad justo debajo de `estop`.
- `fake_robot` publica un `/imu/data` ideal (`imu_rate_hz`, 20 por defecto; 0 = no publica, para probar la pérdida) para que la simulación no dispare la alerta. Escenario [22] de `tools/node_scenarios.py`.
- Límite: si el propio `imu_watchdog` no corre, no hay alerta (falla abierta). Con `nav.launch.py` arranca siempre. Al levantar el stack puede verse un `imu_lost` breve hasta que llega el primer mensaje del IMU.

## Bloque B: cobertura en líneas paralelas (`coverage_parallel`)

Al **entrar** a `auto_parallel` genera la ruta de barrido sobre el cerco y se la da al seguidor; fuera de ese modo no hace nada.

```bash
ros2 launch lawnbot_nav nav.launch.py                           # incluye geofence_node y coverage_parallel
ros2 launch lawnbot_nav waypoint_follower.launch.py config:=map  # seguidor en frame map con auto-inicio
```

- **Entradas:** `/geofence/polygon` (polígono en `map`, del `geofence_node`) y `/nav/active_mode`. **Salidas:** `/follower/path` (frame `map`,
  solo los extremos de cada línea), `/coverage/markers` (`MarkerArray`: cerco, región de trabajo, ruta y puntos, para RViz) y
  `/coverage/status`: `OK lineas=16 longitud=136.8 m tiempo_estimado=12.7 min angulo=0.0 deg espaciado=0.267 m`, `ERROR:<causa>` o `IDLE`.
- **Algoritmo:** la región de trabajo es el cerco recortado `margin` metros hacia adentro. Las líneas son paralelas al lado más largo
  (o a `sweep_angle_deg` si `sweep_angle_auto` es `false`) y van separadas `cutting_width × (1 − overlap)`. El espaciado se ajusta
  levemente hacia abajo para que la última línea caiga justo en el borde interno. Se recorren en zigzag y, con `start_nearest`,
  se empieza por el extremo de línea más cercano al robot (4 formas posibles de recorrerlo).
- **`turn_extension`** (0,15 m): prolonga cada extremo de línea, sin pasar del margen, para que la cuchilla llegue al final real aunque el
  seguidor corte la esquina al darse por llegado (tolerancia por ruta). No puede ser mayor que `margin`.
- **Solo cercos convexos.** En un polígono cóncavo el tramo entre dos líneas podría cruzar fuera del cerco. Un cóncavo da
  `ERROR:poligono no convexo`. Si el margen no deja región (`ERROR:region vacia: un margen de X m no deja espacio (el cerco mide Y m en su lado
  mas angosto; reducir margin)`) o falta el cerco (`ERROR:sin cerco`) tampoco hay ruta. En todos los errores se publica una ruta **vacía**
  (borra cualquier ruta vieja del seguidor) y se avisa con `coverage:coverage_error` por `/nav/alert_in` mientras el modo siga activo.
  Si llega un cerco corregido mientras se espera, se genera en ese momento.
- Al salir de `auto_parallel` (o al apagar el nodo) publica una ruta vacía y borra los markers.
- **Valores provisorios:** `cutting_width` 0,30 m (medir el real) y `margin` 1,0 m (tiene que ser mayor que la dispersión real del GPS).
- `nav.launch.py` **ya no lanza `nav_stub_parallel`** (`use_parallel_stub:=false` por defecto); si no, el patrón a ciegas tomaría el mando
  cuando el seguidor termina o se pausa. `use_parallel_stub:=true` lo vuelve a encender.
- `config/waypoint_follower_map.yaml`: el seguidor en `map` (las rutas por topic deben venir en su mismo frame; el cuadrado de ejemplo en `odom`
  y la cobertura en `map` no pueden convivir en un mismo seguidor).

**Cómo probarlo**
1. Sin ROS: `PYTHONPATH=. python3 test/test_coverage.py` (convexidad, recorte, líneas, extremos, arranque cercano, errores, y el seguidor recorriendo el
   barrido con el robot simulado midiendo el área cubierta).
2. Nodos sin ROS: `python3 tools/node_scenarios.py` (escenario [5]: entrada al modo → ruta → auto-inicio → barrido completo con `fake_robot`).
3. Con ROS y `fake_robot` (sin GPS):
   ```bash
   ros2 launch lawnbot_nav nav.launch.py
   ros2 launch lawnbot_nav waypoint_follower.launch.py config:=map
   ros2 run lawnbot_nav geofence_node --ros-args -p conversion_mode:=datum -p datum_lat:=-25.3 -p datum_lng:=-57.6   # si no corre ya con nav.launch.py
   ros2 run lawnbot_nav fake_robot --ros-args -p frame_id:=map -p x:=0.5 -p y:=0.5 -p cmd_topic:=cmd_vel
   ```
   (con `nav.launch.py` el `geofence_node` ya está; para esta prueba lanzarlo con `ros2 launch lawnbot_nav nav.launch.py geofence_conversion_mode:=datum datum_lat:=-25.3 datum_lng:=-57.6`). Publicar un cerco convexo (ver el
   Bloque A), esperar `/geofence/status` en `OK`, pasar a `auto_parallel` (`ros2 topic pub -r 1 /robot_mode std_msgs/msg/String "{data: 'auto_parallel'}" --qos-durability transient_local --qos-reliability reliable`)
   y mirar `/coverage/status` y `/coverage/markers` en RViz (`Fixed Frame: map`). Con un cerco en L debe dar `ERROR:poligono no convexo`.
4. **Resultado en simulación** (cerco de 6 m × 5 m, `margin` 1,0 m): 13 líneas, 58,9 m; el seguidor lo completa en ~6,5 min cubriendo el
   **100 %** de la región de trabajo (99,6 % con 6 cm de ruido de posición) y sin acercarse al borde del cerco a menos de 0,85 m.

## Bloque C: caminata aleatoria y seguimiento de borde

Los dos publican en su propio topic (`/cmd_vel_random`, `/cmd_vel_perimeter`, entradas de `twist_mux`) y pasan por el `safety_gate`: ninguno
publica en `/cmd_vel`. Solo actúan con su modo activo (`auto_random` / `auto_perimeter`, vía `/nav/active_mode`; sin mensaje de modo en 3 s se
consideran inactivos), publican a 20 Hz, publican cero al desactivarse y al apagarse, y no hacen nada fuera de su modo. `nav.launch.py` los
lanza y apaga los stubs viejos (`use_random_stub` / `use_perimeter_stub` en `false` por defecto: encenderlos junto al nodo real daría dos
publicadores en el mismo topic). Velocidades máximas: 0,2 m/s (random) y 0,15 m/s (perimeter).

### random_walk (`config/random_walk.yaml`)

AVANZAR → (obstáculo, bumper o borde del cerco) → RETROCEDER 0,35 m → GIRAR un ángulo al azar → AVANZAR.

| Disparo | Condición |
|---|---|
| obstáculo | mínimo de los sonares < `obstacle_distance` (0,50 m) |
| bumper | B1, B2, B3 o B4 presionados (B5 corta el retroceso) |
| borde | `/geofence/distance_to_edge` < `edge_distance` (1,5 m) **y** se está acercando; o < `edge_margin` (0,5 m) **y** avanza hacia el borde (la distancia baja, aunque sea en diagonal, o mira hacia afuera del centroide) |

- **Semilla configurable** (`seed`; `-1` = al azar): misma semilla y mismas entradas, misma secuencia de giros.
- **Giros por tiempo** (sin realimentación de rumbo): entre 60° y 180° al azar tras un obstáculo o bumper.
  **Tras el borde del cerco el giro apunta al centroide del polígono** (`/geofence/polygon` + pose en `map`) con una variación al azar de
  `±edge_turn_spread_deg` (30°). Sin polígono o sin pose gira entre 120° y 180° como antes. En simulación (cerco de 9 × 7 m) esto reduce los
  disparos por borde en ~25 % y el robot sale del borde mirando al interior.
- **Falla segura:** con un sensor requerido sin datos (sonares habilitados o B1/B2) por más de `sensor_timeout`, o sin `/geofence/status` `OK` y
  distancia vigente (`require_geofence: true`), **no se mueve** y avisa (`random:sensor_timeout`, `random:geofence_not_ready`). Con `edge_distance`
  se puede arrancar dentro de la franja del borde: solo dispara si se acerca, así sale hacia adentro en vez de retroceder en bucle.
- **Margen del borde (`edge_margin`, 0,5 m):** dentro del margen y avanzando hacia el borde hace la maniobra de borde (retroceso y giro al centroide) **antes de cruzarlo**. Si ya mira al interior no dispara
  (antes disparaba siempre, y el retroceso lo llevaba hacia el borde). El retroceso se corta si el borde queda a menos de `edge_backup_stop_distance` (0,15 m) y se sigue acercando.
- **Fuera del cerco (distancia negativa) → recuperación:** a menos de `max_outside_recovery` (1,0 m) del borde **gira en el lugar hacia el centroide** y **avanza despacio** (`recover_speed`, 0,1 m/s) hasta volver a
  entrar. Si hay un obstáculo adelante (`obstacle_distance` o bumper frontal, lo mismo que frenaría el `safety_gate`) **gira más** antes de avanzar y, al despejarse, avanza recto `recover_clear_time_s` (1,5 s);
  si la cola o un costado rozan (bumper trasero o lateral) **avanza** en vez de girar. A más de `max_outside_recovery`, sin el centroide (`/geofence/polygon` o pose) o pasados `recover_timeout_s` (60 s)
  **se queda quieto y avisa `random:outside_fence`**. Un cerco nuevo que llega con el robot en marcha y lo deja afuera entra en esta recuperación; mientras el cerco se reconvierte (`WAITING_CONVERSION`, distancia NaN)
  sigue quieto (`geofence_not_ready`). Si nunca llegó un cerco o el status no es `OK`, no se mueve (`require_geofence: true`; el log dice `cerco requerido: True`).
- **Cerco restaurado de `~/.ros/lawnbot_geofence.json`:** `geofence_node` lo trata igual que uno recibido por `/geofence_raw` (mismo `accept_raw`; solo no lo vuelve a guardar), en `datum` y en `navsat`
  (en `navsat` queda `WAITING_CONVERSION` hasta que `fromLL` responde, como el recibido). Escenario [21] de `tools/node_scenarios.py`.
- Atrapado (6 disparos en 30 s): alerta `random_blocked`.
- **Límite conocido:** del borde solo se conoce una distancia (no la dirección), así que en una esquina del cerco puede rebotar entre dos bordes un rato.

### perimeter (`config/perimeter.yaml`)

Sigue una pared o borde físico (`side: right|left`) **por contacto con el bumper lateral** (B4 / B3). El sonar lateral es un refinamiento opcional.

**Modo base (bumper, `use_side_sonar: false`):**
0. **Búsqueda inicial del borde** (al activarse el modo, sin pared a la vista): avanza **recto** a `search_speed` (0,15 m/s) hasta el **primer contacto** de un bumper o la **detección del
   sonar central** (`obstacle_distance`; el sonar lateral también cuenta si está habilitado). No gira en círculos. Si se acerca al **borde del cerco virtual** (a menos de `search_edge_margin`, 1 m)
   mirando hacia afuera, gira en el lugar **hacia el interior** (hacia el centroide del cerco ± `search_edge_spread_deg`, como `random_walk`; sin centroide o pose, un giro al azar entre
   `search_edge_turn_min_deg` y `search_edge_turn_max_deg`) y sigue buscando; fuera del cerco también gira hacia adentro. Sin cerco (`/geofence/distance_to_edge` sin dato) no maniobra en ningún borde.
   Solo se rinde pasados **`search_timeout_s` (120 s por defecto)** desde que se activó, con la alerta `perimeter_lost`; los giros hacia el interior no reinician ese plazo.
   Una vez hallada la pared, sigue con los pasos de abajo. `seed` (−1 = al azar) fija los giros al azar para reproducir una corrida.
1. Avanza con una curva suave hacia la pared (`seek_turn`) hasta tocarla.
2. Al tocar **retrocede derecho** `contact_back_distance` (0,10 m a `contact_back_speed`) **antes de girar**, para que la cola no barra la pared.
3. Gira en el lugar alejándose `contact_turn_angle_deg` (15°; `front_contact_turn_angle_deg` = 45° tras un contacto frontal) y vuelve a buscar el contacto.
4. Si al girar el bumper sigue presionado (la cola roza la pared), **avanza** `contact_forward_time_s` en vez de empujar. Si el contacto es continuo
   más de `contact_timeout_s`, intenta liberarse con un retroceso en arco (`unstick_*`, hasta `unstick_attempts` veces) y recién después se declara bloqueado.
- **Esquina interior:** el sonar central (`obstacle_distance`, 0,45 m; se puede apagar con `use_center_sonar: false`) o un bumper frontal disparan el giro.
- **Contacto lateral girando en la esquina:** si en `CORNER` se presiona el bumper lateral, retrocede `corner_back_distance` (0,12 m) antes de seguir girando. Si tras ese
  retroceso sigue tocando (con el rumbo hacia la esquina, retroceder arrastra la cola contra la pared), o si lo que toca es el bumper trasero, **avanza** `contact_forward_time_s`
  a `contact_forward_speed` y vuelve a `CORNER`. El tiempo total en una esquina sigue acotado por `corner_timeout_s` (8 s) → `UNSTICK` → `perimeter_blocked`.
- **Esquina por falta de progreso:** `stalled_contacts` (3) contactos laterales seguidos con menos de `stalled_progress_distance` (0,15 m) de avance entre uno y otro se tratan como
  contacto frontal: retroceso y giro de `front_contact_turn_angle_deg` (45°). Así la esquina se detecta aunque el chasis no presione ningún bumper frontal. `stalled_contacts: 0` lo apaga.
  El avance se mide con la **pose real**: distancia entre las posiciones del robot en dos contactos laterales consecutivos, con la TF `frame_id → base_frame` (`perimeter.yaml`:
  `odom → base_footprint` en el robot; con `fake_robot` y `sim_room.yaml`, que publica `map → base_footprint`, hay que arrancar `perimeter` con `frame_id: map`). Un robot que empuja
  contra la pared sin moverse, que el `safety_gate` frena o que patina da ~0 m aunque se le ordene `v > 0`. **Respaldo:** si la TF falta (o es más vieja que `pose_timeout`, 0,5 s) usa
  el avance neto ordenado (avance menos retroceso, integrando `v·dt`), que **no detecta** un robot trabado; el log avisa una vez por activación (`sin TF odom -> base_footprint: ...`).
  El primer contacto tras activar no tiene contacto previo con que comparar y también usa el respaldo. Ejemplo típico del ciclo toque–retroceso 0,10 m–giro 15° en una esquina sin
  bumper frontal: avanza ~2–3 cm por ciclo.
- **Borde perdido** (una pared ya hallada que se pierde: sin contacto por `bump_lost_timeout_s`, 10 s): búsqueda **acotada** (arco hacia la pared y tramo recto) por
  `lost_search_timeout_s` (25 s, es el `search_timeout_s` de antes: ahora `search_timeout_s` es solo la búsqueda inicial); si no lo reencuentra se detiene y avisa `perimeter_lost`. Otros fallos: `perimeter_blocked` (atascado o frente que no se libera).
- **Todos los parámetros de contacto están en el YAML** para ajustarlos en el robot (ángulos, distancias, velocidades y tiempos).

**Refinamiento con el sonar lateral (`use_side_sonar: true`):** el sonar del lado de la pared mide la distancia lateral (`lectura × sin(corner_sonar_angle_deg)`) y un
control proporcional la mantiene en `target_distance` (0,30 m), con el bumper como contacto de respaldo; las esquinas exteriores se rodean con un arco. **Solo se
usa si `corner_sonar_angle_deg` es válido (≥ 20°)**; si no, se ignora el sonar (queda un aviso en el log) y se sigue por bumper, sin fallar.

- **Probado con dos formas de bumpers en el simulador:** `test_perimeter.py` da la vuelta completa (pared a derecha e izquierda y el cuarto de `sim_room.yaml`) con las zonas de bumper actuales
  de `sim_world.py` y con las anteriores (esquinas compartidas, que hacían que un contacto lateral en la esquina presionara también B2/B5). Sin `perimeter_blocked` en ninguna. Con las zonas
  actuales la vuelta tarda más (~790 s contra ~480 s en `sim_room`) porque la esquina se resuelve por falta de progreso en vez de por el bumper frontal. Con `corner_radius` > 0 (ver abajo) la vuelta también se completa; la disposición real de
  los bumpers en el chasis hay que medirla en el robot.
- **Con la huella real del chasis** (`body_shape: chassis`, esquinas de 60 mm, zona sin bumper de ~55°; `ChassisFootprintTests` en `test_perimeter.py`): el perimetral da la vuelta sin `perimeter_blocked`
  ni choques en `room` (900 s, pared a derecha e izquierda, recorre de 0,2 a 9,8 m en x y de 0,2 a 7,8 m en y) y en `field` (encuentra la pared recta y la sigue de y = 8,4 a 21,7 m; arrancando al oeste rodea la L).
  **Toques sin ningún bumper presionado** (flancos de subida a 5 Hz, contacto a menos de 2 cm; entre paréntesis, los toques que sí presionan un bumper):
  | Mundo (pared, arranque) | Tiempo | Zona muerta de las esquinas | Costados traseros | Escalones |
  |---|---|---|---|---|
  | `room`, derecha | 900 s | 2 (704) | 1 | 0 |
  | `room`, izquierda | 900 s | 0 (704) | 0 | 0 |
  | `field`, derecha, al este | 1500 s | 7 (1234) | 0 | 0 |
  | `field`, derecha, al oeste (la L) | 1500 s | 4 (1054) | 0 | 0 |
  | `field`, izquierda, al este | 1500 s | 2 (1225) | 0 | 0 |
  | `field`, izquierda, al oeste (la L) | 1500 s | 4 (1052) | 0 | 0 |
  Son pocos (≈0–1 % de los toques) pero existen: una esquina que roza la pared dentro de la zona muerta no se entera. El perimetral los resuelve con el contacto del bumper lateral o el frontal siguiente y la
  detección por falta de progreso. El valor depende del tamaño de la zona muerta y del radio, que son los del plano.
- **Limitación (simulación):** con el cuerpo rígido, en una esquina interior pegado a la pared el robot no puede girar sin que la cola roce; el retroceso en arco
  (`unstick_*`) lo resuelve en el cuarto simulado, pero con paredes reales conviene ajustar `contact_*` y `unstick_*` en el robot.
- **IMPORTANTE (a medir):** el ángulo de los sonares de las esquinas (`corner_sonar_angle_deg`, 30° **provisorio**) es **un solo parámetro compartido** (ver *Ángulo de los sonares de las esquinas*, abajo); hace falta medirlo en el robot.

**Ángulo de los sonares de las esquinas (`corner_sonar_angle_deg`, un solo parámetro).** Hay un sonar al centro del frente y uno en cada esquina delantera; el de las esquinas apunta
hacia afuera un ángulo respecto del frente (0° = al frente, 90° = de costado; izquierdo `+`, derecho `−`). Vale **30° provisorio** hasta medirlo y vive **solo** en
`config/robot_geometry.yaml` (`/**: ros__parameters: corner_sonar_angle_deg`). Lo leen `fake_robot` (`sim_world.launch.py`), `potential_field`, `coverage_map` y `perimeter` (`nav.launch.py`;
en `perimeter` es el `side_sonar_angle_deg` de antes), y de ahí sale el yaw de los sonares del URDF:
- `mow_simplified/urdf/mow_simplified.urdf.xacro` es la **fuente** y `mow_simplified.urdf` (el que usan `localization.launch.py`, `display.launch` y `gazebo.launch`) es el **generado**: tras cambiar el ángulo,
  correr `mow_simplified/scripts/generate_urdf.sh` (hace falta `xacro`: ROS 2 o `pip install xacro`) y commitear los dos. `test_robot_geometry.py` falla si el `.urdf` quedó desfasado del YAML.
- **Ruedas motrices:** `wheel_separation` (0,355 m entre centros, igual que `base_width` del firmware) está en el mismo YAML; el xacro pone las ruedas en y = ±`wheel_separation`/2 (±0,1775 m; antes
  ±0,18 m del export de SolidWorks). El simulador usa `base_width` 0,355 (`fake_robot` / `sim_robot`); `sim_world` no modela la colisión de las ruedas (el cuerpo es la huella del chasis).
- Se quitaron `side_sonar_yaw_deg` (fake_robot), `sonar_left_yaw_deg` / `sonar_right_yaw_deg` (coverage_map), el yaw del tercer elemento de `sonar_left` / `sonar_right` (potential_field, que ahora son `[x, y]`) y
  `side_sonar_angle_deg` del YAML de `perimeter`. Antes `coverage_map` usaba 0° (como el URDF) y el resto 45°: ahora todos usan el mismo.

### Pruebas

1. Sin ROS: `PYTHONPATH=. python3 test/test_sim_world.py`, `test_random_walk.py`, `test_perimeter.py` y `test_gate_sonars.py` (uno por vez). `sim_world.py` es un cuarto simulado
   con paredes y obstáculos circulares, sonares (cono de ±7,5°), bumpers por zona del cuerpo y colisiones.

   **Huella del chasis, esquinas y bumpers en el simulador** (`fake_robot`, `config/sim_room.yaml` y `sim_field.yaml`; medidas del plano CAD, ver `lawnbot_nav/robot_geometry.py`):
   | Parámetro | Defecto en el código | En `sim_room.yaml` / `sim_field.yaml` | Significado |
   |---|---|---|---|
   | `body_shape` | `rectangle` | `chassis` | `chassis` = la huella real: **bloque delantero de 400 mm de ancho × 226 mm de profundidad** y **bloque trasero de 250 mm de ancho** (largo total 436 mm; `body_length`, `body_front_depth`, `body_front_width`, `body_rear_width`). `rectangle` = el cuerpo rectangular de antes (para los tests viejos) |
   | `corner_radius` | 0,0 m | 0,06 m | radio de las **esquinas delanteras** (en `chassis`; en `rectangle`, de las cuatro). También en la colisión. Las traseras son rectas |
   | `corner_split_deg` | 45° | 41° | ángulo en el arco, medido desde la cara frontal (0°) hacia la lateral (90°), donde se parte entre el bumper frontal y el lateral |
   | `corner_dead_deg` | 0° | 55° | zona **sin bumper** centrada en el split: de 13,5° a 68,5° no hay bumper (B1/B2 entran en el arco hasta ~13°; B3/B4 empiezan a ~38 mm del frente, ~68° del arco) |

   **Bumpers del chasis:** B1/B2 cubren el frente recto y el arco hasta ~13°; B3/B4 el costado del bloque delantero (desde ~68° del arco hasta el escalón donde empieza el bloque trasero); B5 la cara trasera, centrada. **Sin bumper**: la zona muerta de cada esquina delantera, los costados del bloque trasero y los escalones. `SimWorld.unprotected_contacts(pose)` informa cuál de esas tres zonas toca una pared a menos de 2 cm; `test_perimeter.py` los cuenta (`log['unprotected']`). Supuestos del plano que **no** venían en la cota: el largo de B3/B4 hacia atrás (se toma hasta el escalón) y que B5 cubre toda la cara trasera. El frente queda en x = 0,2137 m de `base_footprint` y la trasera en −0,2223 m (el joint del bumper trasero del URDF sigue en −0,2463: viene de la exportación de SolidWorks y no se tocó).

   Un bumper se presiona con un punto de su zona a ≤ 2 cm de una pared, así que una pared pegada a un costado también toca la parte del arco más cercana a ella: con radio 0,15 m
   y split 45° solo el lateral; si el split es 80° se presiona además B2 o B5. `perimeter` da la vuelta completa de `sim_room` con radio 0,05 y 0,10 m y zona muerta de 0° y 20°
   (`test_perimeter.py`, con la pose real para el avance entre contactos).

   **Mundos simulados** (`sim_world.launch.py world:=room|field`, por defecto `room`):
   | `world` | Archivo | Qué es | Arranque |
   |---|---|---|---|
   | `room` | `config/sim_room.yaml` | cuarto de 10 × 8 m con un bloque y un obstáculo circular | (2, 1) |
   | `field` | `config/sim_field.yaml` | campo abierto de 30 × 30 m con 3 obstáculos chicos (radios 0,25–0,4 m), un borde exterior lejano a 5 m (de −5 a 35 en x e y) para que el robot no se escape y dos barreras para probar `auto_perimeter`: una **pared recta de 12 m** en x = 19 (de y = 9 a 21, 4 m al este del arranque) y una **barrera en L de 3 × 3 m** con la esquina en (9, 13) y brazos hasta (9, 16) y (12, 13). Desde el arranque (mirando al este) el perimetral encuentra la pared recta; para la L arrancar `fake_robot` en (15, 15) con `yaw: 3.14159` (mira al oeste) | (15, 15), el centro |

   Un `world` desconocido hace fallar el launch con un mensaje claro. Los dos usan la misma huella del chasis y las mismas esquinas de arriba.

   **Probar la app con un cerco dibujado a escala real.** Dibujá el cerco en la app (queda en `~/.ros/lawnbot_geofence.json`, en lat/lon) y alinealo con el mundo con
   `tools/datum_from_fence.py`, que calcula `datum_lat`/`datum_lng` para que el centro del cerco (el centroide del polígono) caiga en el centro del mundo (centro de la caja de las paredes):
   ```bash
   python3 ros2_ws/src/lawnbot_nav/tools/datum_from_fence.py --world field        # --fence RUTA.json, --margin 0.4
   # imprime, por ejemplo:
   ros2 launch lawnbot_nav sim_world.launch.py world:=field
   ros2 launch lawnbot_nav nav.launch.py geofence_conversion_mode:=datum datum_lat:=-25.300010407 datum_lng:=-57.599998952
   ```
   Antes de imprimir los comandos comprueba que el cerco sea válido (mismas reglas que `geofence_node`), que **entre en el mundo** (ningún vértice sobre o fuera de las paredes) y que el arranque del
   robot quede dentro del cerco con al menos `--margin` (0,4 m) hasta el borde. Si algo falla imprime el motivo, **no** imprime comandos y sale con código 2 (1 si no puede leer el cerco).
   Con `room` el cerco tiene que caber en 10 × 8 m y contener el arranque (2, 1) con margen. No hace falta ROS. Si `nav.launch.py` ya estaba corriendo con otro datum, reiniciarlo: `geofence_node` no aplica el datum en caliente.

   **Posición del robot en la app (simulación).** `robot/position` sale de gpsd y en simulación no hay GPS, así que el robot no aparece en el mapa de la app. El bridge tiene una opción, **apagada por defecto**,
   que toma la pose de la TF `map → base_footprint`, la convierte a lat/lng con el **mismo datum** que `geofence_node` (modo `datum`, misma `meters_per_degree`, así el robot cae dentro del cerco dibujado) y publica
   `robot/position` a 1 Hz con el JSON de siempre `{"lat", "lng", "hdg"}`. El rumbo `hdg` es como el de gpsd (grados desde el norte, sentido horario; yaw 0 = este = 90°). Con la opción activa **no se conecta a gpsd**
   (ni hace falta tenerlo instalado); si no hay TF o es vieja (> 2 s) no publica nada. Sin la opción el bridge es el de siempre y no necesita `sim_position.py` ni `lawnbot_nav`.
   ```bash
   # datum_lat/datum_lng salen de tools/datum_from_fence.py (que imprime este comando ya armado)
   python3 bridge/bridge.py --sim-position --datum-lat -25.300010407 --datum-lng -57.599998952
   # o por variables de entorno
   LAWNBOT_SIM_POSITION=1 LAWNBOT_DATUM_LAT=-25.300010407 LAWNBOT_DATUM_LNG=-57.599998952 python3 bridge/bridge.py
   ```
   Orden de arranque: `sim_world.launch.py world:=field`, `nav.launch.py geofence_conversion_mode:=datum datum_lat:=… datum_lng:=…` y el bridge con **el mismo datum** (con otro, el robot se ve desplazado respecto del cerco).
   En el portátil hace falta `source` de `lawnbot_nav` (el bridge importa su `geofence_logic` y `tf_utils`) y `bridge.py` junto a `sim_position.py`. No activar la opción en `bridge.service` del robot real.
   Falla al arrancar con un mensaje claro si falta el datum o es inválido (0, 0). Pruebas: `python3 bridge/test_sim_position.py` y el escenario [22] de `tools/node_scenarios.py`.

   **Ver la simulación en RViz** (solo simulación: lo publica `fake_robot`, nada del robot real; en `sim_room.yaml` va `publish_markers: true` y por defecto está apagado):
   ```bash
   ros2 launch lawnbot_nav sim_world.launch.py        # fake_robot (y, aparte, nav.launch.py)
   rviz2 -d $(ros2 pkg prefix lawnbot_nav)/share/lawnbot_nav/config/sim.rviz     # o: rviz2 -d ros2_ws/src/lawnbot_nav/config/sim.rviz
   ```
   `sim.rviz` abre con *Fixed Frame* `map` (el `frame_id` de `fake_robot` en `sim_room.yaml`) y estos displays:
   | Topic | Tipo | Qué muestra |
   |---|---|---|
   | TF | — | `map → base_footprint` |
   | `/sim/markers` | `MarkerArray`, ~1 Hz, `transient_local` | paredes y obstáculos de `sim_room.yaml`; la huella del robot con las esquinas redondeadas (`corner_radius`, en `base_footprint`, así sigue al robot); cada bumper como un segmento, **verde libre y rojo presionado** (al cambiar un bumper se republica enseguida, sin esperar el segundo) |
   | `/sim/path` | `nav_msgs/Path`, ~1 Hz, `transient_local` | rastro del robot: las últimas `path_max_poses` (2000) poses, una cada `path_min_step` (2 cm) o `path_min_yaw` (0,087 rad) |
   | `/geofence/polygon`, `/coverage/markers` | `PolygonStamped`, `MarkerArray` | el cerco y la ruta de cobertura (`nav.launch.py` + `coverage_parallel`) |

   Parámetros de `fake_robot`: `publish_markers`, `markers_rate_hz` (1,0), `path_max_poses`, `path_min_step`, `path_min_yaw`. Los segmentos de bumper salen de las mismas zonas
   que usa la simulación (`corner_split_deg`, `corner_dead_deg`), así que la zona muerta se ve como un hueco entre dos segmentos. La huella y los bumpers se dibujan en `base_footprint`:
   hace falta la TF. **No** correr `fake_robot` junto con la odometría real (ver arriba).
2. Nodos sin ROS: `python3 tools/node_scenarios.py` (escenario [6]: `fake_robot` en un cuarto, los nodos y el `safety_gate` encadenados; [7]: umbrales por sonar e ignorar los laterales).
3. Con ROS y `fake_robot` en un cuarto con paredes y obstáculos:
   ```bash
   ros2 launch lawnbot_nav nav.launch.py
   ros2 launch lawnbot_nav sim_world.launch.py          # fake_robot + sonares y bumpers simulados (config/sim_room.yaml)
   ros2 topic pub -r 1 /robot_mode std_msgs/msg/String "{data: 'auto_random'}" --qos-durability transient_local --qos-reliability reliable
   ros2 topic hz /cmd_vel_random ; ros2 topic echo /cmd_vel
   ```
   Para `auto_random` hace falta `/geofence/status` `OK` (un cerco en `map`, ver Bloque A); sin él el robot no se mueve y `/nav/safety_alert` marca
   `geofence_not_ready`. Para `auto_perimeter`, `'auto_perimeter'` (los sonares de las esquinas están a ±`corner_sonar_angle_deg`, 30°). Cambiar de modo apaga
   el nodo anterior con ceros. **No correr `ekf_local` ni la odometría real a la vez que `fake_robot`.**
4. **Resultado en simulación** (cuarto de 10 m × 8 m con un bloque y un obstáculo): `random_walk` recorre ~20 celdas de 1 m en 5 min sin tocar nada
   y sin salir del cerco; `perimeter` (modo bumper) da la vuelta al cuarto con la pared a la derecha o a la izquierda, rodea un bloque, y con la pared fuera de alcance
   se rinde a los `search_timeout_s` (120 s) con `perimeter_lost`; con el refinamiento del sonar lateral mantiene ~0,3 m.

## Bloque E: control de la cuchilla y mapa de cobertura

```
app ─MQTT robot/cmd/actuator─► bridge ─► /cutter/enable, /trimmer/enable, /blade/force_cut (Bool, 4 Hz) ─┐
/geofence/inside, /vision/grass_present, bumpers B1..B5, /imu/data, /estop, /coverage/cell_passes
                                         └─► blade_controller ──► /cutter/cmd, /trimmer/cmd (Float32, 10 Hz) ──► cutting_motors (sin cambios)
fake_robot / robot ──► coverage_map ──► /coverage/cell_passes (Int32), /coverage/map (JSON) ──► bridge ──► MQTT robot/map
```

### blade_controller (`config/blade_controller.yaml`)

Publica el duty a **10 Hz** (el `cutting_motors` existente corta solo a los 0,5 s sin órdenes, y no se modificó). Cada motor recibe duty > 0 **solo
si se cumplen todas** las condiciones; en cuanto falla una, o un dato se vuelve viejo, se publica **0** (se sigue publicando el 0, así el watchdog
queda alimentado y apagado). Nunca se publica más que `max_duty` del motor.

| Condición | Cuchilla | Trimmer | Motivo que informa `/blade/status` |
|---|---|---|---|
| Pedido vigente por `/cutter/enable` / `/trimmer/enable` (Bool; vence a los 2 s sin refresco; sin pedido, o al arrancar, todo apagado) | ✓ | ✓ | `sin_pedido` |
| `/geofence/inside` `true` | ✓ | ✓ | `fuera_del_cerco`, `geofence_sin_dato` |
| `grass_present` `true` (`/vision/grass_present`); **`force_cut` anula solo esta condición** | ✓ | — | `sin_pasto`, `pasto_sin_dato` |
| Robot no levantado: (B3 y B4) o (B1 y B5) o (B2 y B5) | ✓ | ✓ | `levantado`, `bumpers_sin_dato` |
| Robot no volcado: inclinación del IMU ≤ `max_tilt_deg` (**20°**); apaga de inmediato al superarlo y rearma 3 s después de volver por debajo | ✓ | ✓ | `volcado`, `imu_sin_dato` |
| Sin `/estop` | ✓ | ✓ | `estop` |
| Celda con **menos** de `max_passes` (2) pasadas completadas | ✓ | ✓ | `celda_con_demasiadas_pasadas`, `pasadas_desconocidas` |
| Todos los datos vigentes (`input_timeout` 1,0 s; IMU 0,5 s) | ✓ | ✓ | `*_sin_dato` |

- **Levantado y volcado se mantienen apagados** `lift_hold_s` / `tilt_hold_s` (3 s) después de resolverse (rebote de bumpers). El firmware entra en estop al
  levantar pero **no lo publica en ROS**, por eso se replica la lógica con los bumpers; los 5 bumpers tienen que estar vigentes para poder detectarlo.
- **`max_duty_cutter` / `max_duty_trimmer`:** 0,30 por defecto (conservador; lo fija el usuario cuando mida las tensiones). La fórmula es
  `tensión_nominal_del_motor / tensión_del_pack_cargado_a_tope` (25,2 V): si se cargan `pack_voltage` y `cutter_nominal_voltage` / `trimmer_nominal_voltage`, el tope baja además a
  ese cociente (un motor de 12 V con el pack a 25,2 V no pasa de 0,48). **Dato que falta: tensión nominal de cada motor.**
- **`force_cut`:** `/blade/force_cut` anula **solo** la condición de pasto de la cuchilla (también si el dato de pasto falta o está viejo). **Nunca** anula el pedido,
  el cerco, el levantado, el volcado, el estop, el límite de pasadas ni la vigencia de los demás datos; no cambia nada del trimmer.
- **`grass_present`:** la cámara no está calibrada y nadie publica `/vision/grass_present`; sin dato la cuchilla **no enciende** (el trimmer sí, si el resto se cumple).
  `vision_stub` (`use_vision_stub:=true` en `nav.launch.py`) publica `grass_present: false` y obstáculos vacíos. **`grass_present: true` solo con el argumento explícito del launch**
  `vision_stub_grass_present:=true` (apagado por defecto; pruebas de banco con los motores desconectados) y mientras esté activo el log advierte cada 10 s que no hay detección real.

### Bridge: de la app a los motores

Payload **real** de la app (`robot/cmd/actuator`, QoS 1, uno por cada cambio; la app no lo refresca): `{"blade": bool, "trimmer": bool, "charging": bool}`. El contrato no cambió.
`force_cut` (bool) es una **clave opcional nueva**: la app todavía no la envía (falta agregarla en `ActuatorCommand`); si falta o no es el booleano `true`, vale `false`.
`charging` no se atiende. La lógica está en `bridge/actuator_relay.py` (**hay que copiarlo a la Pi junto a `bridge.py`**, en `~/pi/`).

- El bridge **republica a 4 Hz** `/cutter/enable`, `/trimmer/enable` y `/blade/force_cut` mientras el pedido siga vigente; `blade_controller` los vence a los 2 s sin refresco (así, si el bridge muere, se apaga).
- **Solo con un modo vigente que lo permita:** `manual` o automático (`auto_parallel`, `auto_random`, `auto_perimeter` según `/nav/active_mode`). Se apaga y se **borra** al pasar a `idle` o `stop`, y si
  se pierde el heartbeat del modo (3 s sin `/nav/active_mode`). Pasar de manual a automático (o al revés) **no** lo apaga. Tras apagarse por esas causas **sigue apagado** hasta que la app pida de nuevo. Arranca siempre apagado.
  Un pedido que llega en `idle`/`stop` se ignora. Solo el booleano `true` enciende (`"true"`, `1` o `null` no cuentan).
  En manual `blade_controller` sigue aplicando todo: cerco, levantado, volcado, estop, datos vigentes y pasto (que solo `force_cut` anula).
- **`robot/status`** (contrato intacto, claves nuevas): `blade` y `trimmer` = lo pedido y aceptado (la app arma su próximo comando a partir de ellos, así que tienen que estar), `force_cut`,
  y `blade_running` / `trimmer_running` = el motor corre de verdad (`/cutter/cmd` o `/trimmer/cmd` > 0 y vigente; puede estar en `false` con el pedido en `true` si una condición lo apaga).
  `blade_status` (clave nueva, opcional): el texto de `/blade/status` (p. ej. `cutter=OFF(sin_pasto) | trimmer=ON duty=0.30`) para que la app diga **por qué** un motor pedido no corre; se omite si tiene más de 3 s.

### coverage_map (`config/coverage_map.yaml`)

Grilla de **0,5 m** (parámetro `resolution`) sobre la caja del cerco (+1 m). Mantiene, por celda: **pasadas completadas**, **certeza de obstáculo por sonar** (un eco sube la certeza de la celda; el
espacio libre delante del eco la baja) y **certeza por cámara** (`/vision/obstacles`, `PoseArray` en `base_footprint`).
- **Pasada:** una visita a la celda durante la cual la cuchilla estaba cortando (`/cutter/cmd > 0`; `count_only_when_cutting: false` cuenta todas), que se cuenta **al salir**
  de la celda (con histéresis, así el vaivén en el borde no suma). `/coverage/cell_passes` informa las completadas **sin** contar la visita en curso, y `-1` si se desconoce.
- **`robot/map` (MQTT):** el bridge reenvía `/coverage/map` (cada 2 s, QoS 0, retenido). JSON: `frame`, `resolution`, `origin [x0, y0]`, `width`, `height`, `encoding`
  (`base64-u8-row-major`), `passes` (0–255), `obstacles_sonar` y `obstacles_camera` (certeza 0–100), `robot [x, y, yaw]` y `stamp`. Cada capa son `width × height` bytes en base64,
  fila por fila desde el origen.
  **`origin_ll`** `[lat, lng]` (clave nueva, opcional): lat/lng del origen (0, 0) del frame `map`, con x = este e y = norte, para que la app ubique la grilla. Sale del servicio **`/toLL`** de `navsat_transform`
  (`robot_localization/srv/ToLL`, pide `map_point`, responde `ll_point`; existe en Humble, verificado en el código fuente de `navsat_transform.cpp`, no en la Pi). Si todavía no hay transformación responde (0, 0): se descarta y se reintenta.
  Se reconsulta cada `origin_refresh_s` (10 s) y, si cambia más de `origin_change_deg` (1e-6°, ~0,1 m; p. ej. otro datum), el mapa se reenvía de inmediato. Sin dato la clave se omite.
  Parámetro `origin_source`: `navsat` (por defecto), `param` (`origin_lat`/`origin_lng` fijos, para pruebas) o `none`. **Hay que comprobar en la Pi** `ros2 service list | grep -i ll` y `ros2 service call /toLL robot_localization/srv/ToLL "{map_point: {x: 0.0, y: 0.0, z: 0.0}}"`. Para un cerco de 42 × 32 m a 0,5 m son ~25 KB por mensaje. Los topics existentes de MQTT no cambian.

- **Nota sobre el error de GPS:** con 2–3 m de error, una celda de 0,5 m no es "exactamente la celda" por donde pasó el robot: el conteo de pasadas es confiable como **tendencia**
  (zonas cortadas de más o de menos), no al centímetro, y la posición del propio robot en el mapa tiene ese mismo error. Una celda menor de 0,5 m no mejora nada y el nodo lo advierte en el log;
  con RTK se puede bajar. Por la misma razón el límite de pasadas (`max_passes`) conviene tomarlo con margen.

### App (pestaña Cerco y Actuadores)

- **Cerco:** dibuja `robot/map` sobre el mapa (capas Pasadas / Sonar / Cámara con interruptores, rotulado como *tendencia* por el error de GPS). Necesita `origin_ll`; sin él avisa que no puede ubicar la grilla. Se asume el frame `map` con x = este e y = norte.
- **Actuadores:** cuchilla y trimmer se pueden pedir en manual y en automático (deshabilitados en idle/stop), muestran *Pedido* y *Corriendo* por separado y, si no corre, el motivo (`blade_status`). *Cortar sin detectar pasto* (`force_cut`) pide confirmación.
- **Batería y alertas:** sin dato de batería la app muestra `--`; `alert` se traduce a español en un banner (`mobile_app/lawnbot_app/lib/models/alert_info.dart`). Tests: `flutter test` en `mobile_app/lawnbot_app`.

### Pruebas (el motor real no se conecta)

1. Sin ROS: `PYTHONPATH=. python3 test/test_blade.py` y `test/test_coverage_map.py` (en `ros2_ws/src/lawnbot_nav`), y `python3 bridge/test_actuator_relay.py`.
2. Nodos sin ROS: `python3 tools/node_scenarios.py` (escenario [8]: con sensores simulados verifica que `/cutter/cmd` pasa a 0 en cada condición, con datos viejos y con el pedido vencido, que
   `force_cut` anula solo el pasto, y que la tercera pasada por la misma línea ya no corta; [9]: el bridge real con MQTT simulado, de `robot/cmd/actuator` a los `Bool`, y `vision_stub`).
3. Con ROS, sin motores (no lanzar `cutting_motors` ni `hardware.launch.py`):
   ```bash
   ros2 launch lawnbot_nav nav.launch.py use_vision_stub:=true
   ros2 launch lawnbot_nav sim_world.launch.py        # robot simulado; hace falta un cerco OK (Bloque A) para tener inside y celdas
   ros2 topic echo /blade/status
   ros2 topic echo /cutter/cmd                         # 0.0 mientras no haya pedido
   # pedido a mano (el bridge lo hace solo desde la app); hay que refrescarlo, vence a los 2 s:
   ros2 topic pub -r 4 /cutter/enable std_msgs/msg/Bool "{data: true}"
   ```
   Con `grass_present` en `false` (`vision_stub` por defecto) la cuchilla queda en 0 (`sin_pasto`); `ros2 topic pub -r 4 /blade/force_cut std_msgs/msg/Bool "{data: true}"` la deja cortar sin pasto
   (solo esa condición). Para la prueba de banco con pasto: `ros2 launch lawnbot_nav nav.launch.py use_vision_stub:=true vision_stub_grass_present:=true`.
   Desde la app: modo automático + interruptor de la cuchilla en la pestaña Actuadores (en MQTT: `mosquitto_pub -t robot/cmd/actuator -m '{"blade":true,"trimmer":false,"charging":false}'`).
   Simular cada condición: `/estop` `true`, `/geofence/inside` fuera del cerco, dos bumpers `true` a la vez (B3 y B4), un `Imu` inclinado, o cortar un publicador (dato viejo).
   Ver el mapa por MQTT: `mosquitto_sub -t robot/map -C 1`.

## Visión (`lawnbot_vision`)

Nodo `vision_node` (`ros2_ws/src/lawnbot_vision`, `config/vision.yaml`). Reemplaza a `vision_stub`: **no correr los dos a la vez** (publican los mismos topics; `nav.launch.py` solo lanza el stub con `use_vision_stub:=true`).
Usa el mismo modelo y las mismas fórmulas que los scripts de la Pi (`vision/pi_scripts/`), sin cambiar el modelo.

```bash
ros2 launch lawnbot_vision vision.launch.py
ros2 topic echo /vision/status        # ok pasto=0.31 obstaculos=2 | sin_datos | SIN_CALIBRAR
```

- **`/vision/grass_present`** (Bool, 5 Hz): hay pasto si la máscara de `pasto` ocupa ≥ `min_grass` (10 %) de la imagen en la mayoría de los últimos `grass_window` (7) cuadros. Es "hay pasto en alguna parte del cuadro", **no** "hay pasto bajo la cuchilla". `blade_controller` lo exige vigente (1 s).
- **`/vision/obstacles`** (`PoseArray`, frame `base_footprint`, x adelante, y izquierda): un punto por máscara de `arbol`, `planta`, `barrera` u `otro`, en su punto de apoyo (fila más baja). Distancia por interpolación de `calib_y` / `calib_d`; ángulo por columna y `fov_h_deg`; luego el montaje de la cámara (`cam_x`, `cam_y`, `cam_yaw_deg`).
  Un objeto más lejos que el punto más lejano calibrado **no se publica** (no se sabe cuánto); más cerca que el más cercano se toma a esa distancia mínima. Lo consumen `coverage_map` y `potential_field` (con `use_vision: true`).
- **Falla segura:** sin cámara, sin modelo, con un cuadro fallido o con un resultado de más de `stale_s` (1 s) publica `grass_present=false` y obstáculos vacíos (la cuchilla se apaga). Al cerrarse hace lo mismo. Si no puede abrir el modelo o la cámara lo dice en el log y sigue publicando "sin datos".
- **`calibrated: false` por defecto:** mientras sea `false`, `/vision/obstacles` sale **vacío** (las distancias de ejemplo no son reales) y el log avisa. Para activarlo: correr `calibrar.py`, copiar `CALIB_Y` / `CALIB_D` a `calib_y` / `calib_d` (con decimales: `300.0`), medir el campo de visión y el montaje de la cámara (el URDF no la tiene), y poner `calibrated: true`.
  Una tabla inválida (`calib_y` no creciente, `calib_d` no decreciente) impide arrancar el nodo.
- **En la Pi:** `ultralytics` y `opencv` se instalan con pip, no con rosdep; el nodo corre con el Python donde estén instalados (probar `python3 -c "import ultralytics, cv2"` con el entorno de ROS cargado). `max_fps: 5.0` es un tope inicial: medir el consumo de CPU junto al resto de los nodos.
  La cámara la abre un solo proceso: **no** correr `pi_stream.py` a la vez.
- Pruebas sin ROS: `cd ros2_ws/src/lawnbot_vision && PYTHONPATH=. python3 test/test_vision_logic.py` y el escenario [14] de `tools/node_scenarios.py`. `colcon build --packages-select lawnbot_vision`.

## Bloque D: evasión por campo de potencial (`potential_field`)

Nodo `potential_field` (`config/potential_field.yaml`, arranca con `nav.launch.py`) que **corrige la orden del algoritmo activo** con la repulsión de los obstáculos cercanos. No planifica rutas.

- **Apagado por defecto.** Se activa por servicio o por topic (solo tiene efecto en `auto_parallel` y `auto_random`; en `auto_perimeter`, `manual`, `idle` y `stop` no publica):
  ```bash
  ros2 service call /potential_field/set_enabled std_srvs/srv/SetBool "{data: true}"
  ros2 topic pub --once /potential_field/enable std_msgs/msg/Bool "{data: true}"
  ros2 topic echo /potential_field/status      # off | waiting_mode | on | avoiding | yield
  ```
- **Cadena:** lee la orden del algoritmo (`/cmd_vel_waypoint` y `/cmd_vel_parallel` en parallel, `/cmd_vel_random` en random), publica `/cmd_vel_potential_field` a 20 Hz, que entra a `twist_mux` con **prioridad 60** (sobre waypoints 50, bajo manual 100) y de ahí al `safety_gate`. Nunca publica en `/cmd_vel`.
  Si el nodo se cae, el mux vuelve solo al algoritmo a los 0,5 s. Al desactivarse, salir del modo o apagarse publica ceros.
- **Reglas:** nunca crea movimiento (orden nominal 0 → 0), nunca supera la velocidad lineal nominal, no retrocede por su cuenta, y el giro agregado se limita a `max_turn` (0,6 rad/s). Obstáculo justo de frente: desempata girando hacia `tie_turn`.
  Sin lecturas vigentes de los tres sonares no publica (el gate frena al algoritmo por sensor viejo).
- **Entradas:** los tres sonares (montaje `sonar_*` en el YAML; **el yaw de los sonares de las esquinas es `corner_sonar_angle_deg` (30°, provisorio), el mismo parámetro que lee `perimeter`, `coverage_map`, `fake_robot` y el URDF**) y, **solo con `use_vision: true`**, `/vision/obstacles` (`PoseArray` en `base_footprint`; en otro frame se ignora con aviso).
  `vision_stub` ya publica ese topic vacío, así que el contrato existe aunque no haya cámara. Lecturas sin eco (>= `sonar_max_range` − 0,05) no son obstáculo.
- **Mínimo local:** con obstáculos dentro de `influence_distance` y orden de avanzar, si el robot no se mueve `stall_min_progress` (0,20 m) en `stall_window_s` (6 s) avisa **`potential_field:local_minimum`** por `/nav/alert_in`
  (llega a la app en `alert`) y **cede el control** `yield_s` (4 s) para que el algoritmo de abajo (p. ej. el retroceso de `random_walk`) resuelva. El progreso se mide con la TF `odom` -> `base_footprint`; sin ella no se detecta y se avisa en el log.
- **Solo publica movimiento con el modo y la orden vigentes:** nada en `auto_perimeter`, `manual`, `idle`, `stop` ni modos desconocidos (a lo sumo ceros al salir de un modo), y deja de publicar si la orden del algoritmo que usa como entrada tiene más de 0,5 s (`nominal_timeout`) o si `/nav/active_mode` tiene más de 3 s. Escenario [12].
- Pruebas sin ROS: `PYTHONPATH=. python3 test/test_potential_field.py` y el escenario [11] de `tools/node_scenarios.py` (random_walk + potential_field + mux + gate con `fake_robot` y obstáculos; robot trabado para el mínimo local).
  Con ROS: `ros2 launch lawnbot_nav sim_world.launch.py`, activar el nodo y pedir `auto_random`.

## Eventos del robot (`robot_events`)

El nodo `robot_events` (`config/robot_events.yaml`, arranca con `nav.launch.py`) publica en **`/robot/events`** (`std_msgs/String`, QoS reliable, volátil) el **nombre del evento**, una sola vez por cambio (flanco):

| Evento | Cuándo |
|---|---|
| `lifted` | dos bumpers opuestos a la vez (B3+B4, B1+B5, B2+B5); un levantado = un evento aunque los bumpers reboten (`lift_hold_s` 3 s) |
| `cutting_started` | `/cutter/cmd` o `/trimmer/cmd` pasan de 0 a > 0 (motor realmente pedido, no solo el pedido de la app) |
| `navigation_started` | el modo activo pasa a `auto_parallel` o `auto_random` desde otro modo (parallel <-> random no repite) |
| `perimeter_started` | el modo activo pasa a `auto_perimeter` |
| `low_battery` | batería < `low_battery_threshold` (20 %) en `low_battery_samples` (3) lecturas seguidas |

`low_battery` tiene histéresis: avisa **una vez** y solo se rearma al subir a umbral + `low_battery_margin` (25 %); con `low_battery_repeat_s` > 0 hay recordatorio periódico (0 = no).
Sin dato válido de batería (0 V = INA228 sin conectar) no se avisa. El primer dato tras arrancar el nodo no genera eventos (reiniciar no dispara nada falso).

- **App:** el bridge pone el evento en `robot/status.alert` durante 5 s, **solo si no hay una alerta de seguridad** (`/nav/safety_alert` manda). Contrato intacto. `bridge/status_alert.py` hay que copiarlo a la Pi junto a `bridge.py` (ver *Despliegue del bridge* en Pendientes).
- **Buzzer (quien lo maneje):** suscribirse a `/robot/events` (`ros2 topic echo /robot/events`) y mapear cada nombre a un patrón de sonido. **No se tocó el firmware**: hoy el firmware no escucha este topic.
- Pruebas sin ROS: `PYTHONPATH=. python3 test/test_events.py`, `python3 bridge/test_status_alert.py` y el escenario [10] de `tools/node_scenarios.py`.

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
- **Despliegue del bridge:** copiar **todos** los `.py` de `bridge/` a la Pi, junto a `bridge.py` (`~/pi/`): `bridge.py`, `actuator_relay.py`, `move_mapping.py` y `status_alert.py` (los tests `test_*.py` no hacen falta). Con backup del `bridge.py` anterior, y luego
  `sudo systemctl restart bridge.service`. Si falta un auxiliar, `bridge.py` **no arranca** y dice cuál falta (`journalctl -u bridge.service -n 20`).
  ```bash
  scp bridge/bridge.py bridge/actuator_relay.py bridge/move_mapping.py bridge/status_alert.py mow@<ip-de-la-pi>:~/pi/
  ```
  Hasta entonces la Pi sigue con el bridge viejo.
