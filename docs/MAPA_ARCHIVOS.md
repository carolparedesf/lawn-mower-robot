# Mapa de archivos del repositorio

Guía rápida para ubicarse en el repo: qué hay en cada carpeta y qué hace cada archivo importante.
Lo marcado **(viejo / no se usa)** se deja por historia o como respaldo, pero no forma parte del robot que
anda hoy. No hay que borrarlo.

## Cómo está armado el robot (en una línea)

App del celular → (MQTT por WiFi) → `bridge/bridge.py` en la Raspberry Pi → ROS 2 (navegación, seguridad,
visión, localización) → `micro_ros_agent` (cable USB) → firmware del ESP32 → motores y sensores.

## Árbol de carpetas

No se muestran las carpetas que se generan solas (`build/`, `install/`, `log/`, `.pio/`, `__pycache__/`)
ni las carpetas de plataforma de Flutter (`android/`, `ios/`, `linux/`, `macos/`, `windows/`, `web/`,
`.dart_tool/`).

```
lawn-mower-robot/
├── README.md
├── bridge/                      Puente app (MQTT) <-> ROS 2, corre en la Pi
├── docs/                        Instructivos y documentación
│   └── hardware/                Reglas udev de los puertos USB
├── firmware/                    Programa del ESP32 (PlatformIO, C++)
│   ├── src/                     Programas principales (uno por "env")
│   │   └── test_sketches/       Pruebas sueltas de la batería y la pantalla
│   ├── lib/                     Librerías propias: motores, sensores, pantalla...
│   │   ├── battery/  bumper/  buzzer/  diffdrive/  display/
│   │   ├── encoder/  motor/  pid/  sonar/
│   │   ├── shared_state/        Datos compartidos entre tareas (solo mow_tasks)
│   │   └── task_logic/          Lógica pura de las tareas (solo mow_tasks)
│   ├── include/                 (vacía, plantilla de PlatformIO)
│   └── test/host/               Tests del firmware que corren en la PC
├── mobile_app/
│   ├── lawnbot_app/             App Flutter para manejar el robot
│   │   ├── lib/                 Código de la app
│   │   │   ├── config/  models/  providers/  screens/
│   │   │   ├── services/  theme/  utils/  widgets/
│   │   ├── test/                Tests de la app
│   │   ├── tools/               Robot falso para probar la app
│   │   └── docs/ref/            Imagen de referencia del diseño
├── mow_simplified/              Modelo 3D del robot (URDF) exportado de SolidWorks
│   ├── urdf/  meshes/  launch/  config/  scripts/
├── pi_setup/                    Configuración de la Raspberry Pi (ventilador)
├── ros2_ws/                     Workspace de ROS 2 (lo que corre en la Pi)
│   └── src/
│       ├── lawnbot_nav/         Navegación, seguridad, cerco, cuchilla, simulador
│       │   ├── lawnbot_nav/     Código de los nodos
│       │   ├── launch/  config/  tools/  test/
│       ├── lawnbot_localization/  Dónde está el robot (filtros EKF + GPS)
│       ├── lawnbot_vision/      Cámara + modelo de segmentación (pasto y obstáculos)
│       ├── lawnbot_hardware/    Motores de corte y sensor TF-mini
│       ├── berryimu_gps/        IMU (BerryIMU) y GPS
│       └── mow_simplified ->    Enlace a ../../mow_simplified (para que ROS encuentre el URDF)
└── vision/                      Modelos de visión y scripts sueltos de prueba
    ├── models/ncnn_320/         Modelo que usa el robot
    ├── models/ncnn_640/         Modelo más grande
    └── pi_scripts/              Scripts sueltos (antes de ROS)
```

## Carpetas y archivos

### Raíz

- `README.md`: presentación del proyecto. **(viejo: la estructura de carpetas que muestra ya no es la real;
  usar este mapa)**.
- `.gitignore`: qué archivos no se suben a git.

### `bridge/` — puente entre la app y ROS 2

Corre en la Pi como servicio (`bridge.service`). Traduce los mensajes MQTT de la app a topics de ROS 2 y al revés.

- `bridge.py`: el puente. Recibe órdenes de la app (mover, modo, e-stop, cerco, cuchilla) y publica el estado del
  robot, la posición y el mapa para la app.
- `fused_position.py`: de dónde sale la posición que ve la app: la del filtro (EKF + GPS) pasada a lat/lng; si no
  está, el GPS crudo de `gpsd`. También arma el mensaje del log que dice cuál falta y por qué.
- `move_mapping.py`: convierte la orden del joystick `{dir, speed}` en velocidad lineal y angular.
- `estop_latch.py`: recuerda si el e-stop está activo, para informarlo a la app.
- `status_alert.py`: elige qué alerta se muestra en la app cuando hay varias.
- `actuator_relay.py`: arma los pedidos de la app para los motores de corte.
- `sim_position.py`: posición GPS falsa a partir de la simulación (solo para pruebas).
- `test_*.py`: tests de los archivos anteriores (uno por archivo).

### `docs/` — instructivos

- `MAPA_ARCHIVOS.md`: este archivo.
- `setup_navegacion.md`: **el documento principal de la navegación**: nodos, topics, cómo probar cada modo, cerco,
  cuchilla y simulador.
- `setup_ssh.md`: cómo entrar a la Pi por SSH y comandos útiles (servicios, logs).
- `setup_usb.md`: nombres fijos para los puertos USB del ESP32 y del TF-mini (`/dev/esp32`, `/dev/tfmini`).
- `setup_wifi.md`: cómo agregar una red WiFi a la Pi.
- `setup_mapa.md`: cómo ver el robot en el mapa.
- `hardware/99-lawnbot-serial.rules`: la regla udev que se copia a la Pi (ver `setup_usb.md`).

### `firmware/` — programa del ESP32

Se compila y se carga con PlatformIO desde la laptop. Cada "env" de `platformio.ini` compila un programa distinto.

- `platformio.ini`: lista de envs: `mow_tasks` (el actual), `mow` (respaldo), `microros_min`, `test_battery`,
  `test_display`.
- `MOW_TASKS.md`: explica el firmware en tareas: cómo cargarlo, topics nuevos y la seguridad.
- `colcon_tasks.meta`: amplía la cantidad de publicadores y suscripciones de micro-ROS para `mow_tasks`.
- `src/main_tasks.cpp`: **el firmware actual** (env `mow_tasks`). Lo organiza en 3 tareas: control de motores a
  50 Hz, sensores a 20 Hz (sonares, bumpers, batería, pantalla, buzzer) y comunicación con ROS (micro-ROS).
- `src/main.cpp`: firmware anterior, todo en un solo lazo (env `mow`). **(viejo / no se usa: queda de respaldo)**.
- `src/microros_min.cpp`: prueba mínima de micro-ROS, sin sensores ni motores. Sirve para descartar problemas del
  ESP32 o del cable (env `microros_min`).
- `src/test_sketches/test_bat_manager.cpp`: prueba suelta del medidor de batería (env `test_battery`).
- `src/test_sketches/test_display.cpp`: prueba suelta de la pantalla OLED (env `test_display`).
- `lib/motor/`: manejo de un motor (PWM y dirección).
- `lib/encoder/`: lectura de los encoders de las ruedas.
- `lib/pid/`: controlador PID de velocidad de cada rueda.
- `lib/diffdrive/`: tracción diferencial: de velocidad del robot a velocidad de cada rueda, y la odometría.
- `lib/sonar/`: los 3 sensores de ultrasonido.
- `lib/bumper/`: los 5 bumpers (paragolpes).
- `lib/battery/`: tensión y corriente de la batería (INA228).
- `lib/buzzer/`: sonidos del buzzer.
- `lib/display/`: pantalla OLED.
- `lib/shared_state/`: datos que comparten las tareas de `mow_tasks`, protegidos para que no se pisen.
- `lib/task_logic/TaskLogic.h`: reglas de `mow_tasks` sin hardware (cuándo se habilitan los motores, detección
  de levantado, sonidos): se pueden probar en la PC.
- `test/host/test_task_logic.cpp`: tests de `TaskLogic.h` que corren en la PC con `g++`.
- `.vscode/`: configuración del editor (sus archivos temporales están en `.gitignore`).

### `mobile_app/lawnbot_app/` — app del celular (Flutter)

Se conecta por MQTT al `bridge.py` de la Pi.

- `README.md`, `ARQUITECTURA.md`: cómo correr la app y cómo está organizada por dentro (explica cada archivo en
  detalle).
- `CLAUDE.md`: instrucciones para el asistente de código.
- `pubspec.yaml`: dependencias de la app. `analysis_options.yaml`: reglas de estilo.
- `lib/main.dart`: arranque de la app.
- `lib/config/mqtt_config.dart`: dirección del broker MQTT y nombres de los topics.
- `lib/services/mqtt_service.dart`: conexión MQTT (conectar, reconectar, enviar y recibir).
- `lib/providers/`: estado de la app: conexión (`connection_provider`), estado del robot (`robot_state_provider`),
  cerco (`geofence_provider`), mapa de cobertura (`coverage_map_provider`), motores de corte
  (`actuator_provider`) y límite de velocidad (`speed_limit_provider`).
- `lib/models/`: formatos de los mensajes: estado del robot, posición, órdenes, alertas, cuchilla, cerco, mapa.
- `lib/screens/`: pantallas: inicio con mapa y joystick (`control_screen`), modo automático (`mode_screen`), cerco
  (`geofence_screen`), actuadores (`actuators_screen`) y la que las contiene (`home_screen`).
- `lib/widgets/`: piezas de pantalla: joystick, botón STOP, liberar e-stop, alertas, batería, marcador del robot,
  capa de cobertura, herramientas del cerco, barras de navegación.
- `lib/utils/`: cálculos de coordenadas (`geo`), controles del polígono del cerco (`polygon_checks`) y nombres de
  los modos (`mode_labels`).
- `lib/theme/robo_theme.dart`: colores y estilo.
- `tools/fake_robot.py`: robot falso por MQTT para probar la app sin el robot.
- `test/`: tests de la app (uno por pantalla o parte), con datos de ejemplo en `test/fixtures/`.

### `mow_simplified/` — modelo del robot (URDF)

Exportado de SolidWorks. ROS lo usa para saber dónde está montado cada sensor (vía `robot_state_publisher`).

- `urdf/mow_simplified.urdf.xacro`: **el que se edita** (fuente del modelo).
- `urdf/mow_simplified.urdf`: generado a partir del `.xacro` (no editarlo a mano).
- `urdf/mow_simplified.csv`: masas e inercias que exportó SolidWorks.
- `scripts/generate_urdf.sh`: regenera el `.urdf` desde el `.xacro`.
- `scripts/joint_state_from_odom.py`: calcula el giro de las ruedas a partir de la odometría, para verlas girar en
  RViz.
- `launch/display.launch`: muestra el modelo en RViz.
- `config/joint_names_mow_simplified.yaml`: nombres de las articulaciones (exportado).
- `meshes/`: piezas 3D (STL).
- `CMakeLists.txt`, `package.xml`: definición del paquete ROS.

### `pi_setup/` — Raspberry Pi

- `fan_control.py`: prende y apaga el ventilador de la Pi según la temperatura.
- `fan-control.service`: servicio para que el ventilador arranque solo.

### `ros2_ws/src/lawnbot_nav/` — navegación y seguridad

Es el paquete más grande. Cada nodo `xxx.py` tiene su lógica en `xxx_logic.py`, sin ROS, para poder probarla aparte.

Cadena de las órdenes de movimiento: algoritmos y app → `twist_mux` (elige por prioridad) → `/cmd_vel_mux` →
`safety_gate` → `/cmd_vel` → ESP32.

**Launch**
- `launch/nav.launch.py`: **arranca toda la navegación**: modos, `twist_mux`, `safety_gate`, cerco, mapa de
  cobertura, cuchilla, eventos y, opcional, la visión (`vision:=true`).
- `launch/waypoint_follower.launch.py`: arranca el seguidor de rutas (`config:=map` en el robot).
- `launch/sim_world.launch.py`: robot simulado con sonares y bumpers en un mundo con paredes, para probar sin
  hardware.
- `launch/follower_sim.launch.py`: prueba del seguidor con un robot simulado simple.

**Seguridad y coordinación**
- `safety_gate.py` / `safety_logic.py`: última barrera antes del ESP32. Frena o bloquea por e-stop, bumpers,
  obstáculos cerca, fuera del cerco o sensores caídos.
- `mode_manager.py` / `mode_logic.py`: decide qué modo está activo (manual, paralelo, aleatorio, perimetral) y lo
  avisa a todos.
- `geofence_node.py` / `geofence_logic.py`: cerco virtual: convierte el polígono de la app (lat/lon) al mapa del
  robot, dice si está adentro y avisa si se acerca a menos de 0,5 m del borde.
- `imu_watchdog.py`: avisa si el IMU deja de publicar y calcula la inclinación para la cuchilla.
- `sonar_filter.py` / `sonar_filter_logic.py`: limpia las lecturas de los sonares.
- `robot_events.py` / `events_logic.py`: eventos del robot (levantado, empezó a cortar, etc.) para el buzzer y la
  app.

**Algoritmos de movimiento** (solo trabajan cuando su modo está activo)
- `coverage_parallel.py` / `coverage_logic.py`: modo paralelo: arma una ruta en líneas paralelas dentro del cerco.
- `waypoint_follower.py` / `pure_pursuit.py` / `auto_start.py`: sigue esa ruta punto a punto.
- `random_walk.py` / `random_walk_logic.py`: modo aleatorio: avanza, y al encontrar algo retrocede y gira.
- `perimeter.py` / `perimeter_logic.py`: modo perimetral: sigue el borde tocándolo con el bumper lateral.
- `potential_field.py` / `potential_field_logic.py`: esquiva obstáculos (sonares y, opcional, cámara). Arranca
  apagado.
- `active_resources.py`: hace que los algoritmos no gasten CPU cuando no están activos.

**Cuchilla y mapa**
- `blade_controller.py` / `blade_logic.py`: prende los motores de corte solo si es seguro (hay pasto, adentro del
  cerco, sin vuelco, etc.).
- `coverage_map.py` / `coverage_map_logic.py`: mapa de cuántas veces pasó por cada zona y dónde hay obstáculos.

**Utilidades compartidas**
- `node_utils.py`: arranque y apagado comunes (al cerrar, los nodos mandan velocidad cero).
- `tf_utils.py`: pedir la posición del robot.
- `odom_pose.py`: la posición del robot desde `/odometry/global` (10 Hz), más barata que escuchar `/tf` (25 Hz) en la Pi.
  La usan `coverage_map` y `geofence_node`; el bridge hace lo mismo con su propio código.
- `sensor_cache.py`: últimas lecturas de sensores con su antigüedad.
- `robot_geometry.py`: medidas del chasis y posición de los sonares.

**Simulación** (para probar sin el robot)
- `fake_robot.py`, `sim_robot.py`, `sim_world.py`, `sim_view.py`: robot y mundo simulados, y su dibujo en RViz.

**Reemplazo de prueba**
- `vision_stub.py`: reemplaza a la cámara (publica "sin pasto, sin obstáculos"). Solo con `use_vision_stub:=true`.

**Configuración (`config/`)**: un `.yaml` por nodo, con el mismo nombre (`safety_gate.yaml`, `random_walk.yaml`,
etc.). Además:
- `twist_mux.yaml`: prioridad de cada fuente de movimiento (la app manual gana sobre los automáticos).
- `robot_geometry.yaml`: ángulo de los sonares y separación de las ruedas (compartido por varios nodos).
- `waypoint_follower_map.yaml`: seguidor en el robot real; `waypoint_follower.yaml`: con un cuadrado de ejemplo.
- `sim_room.yaml`, `sim_field.yaml`, `sim.rviz`: mundos del simulador y la vista de RViz.

**Herramientas (`tools/`)**
- `fake_ros.py` + `node_scenarios.py`: prueban los nodos completos sin ROS instalado (escenarios).
- `datum_from_fence.py`: calcula el punto de origen para que el cerco de la app caiga en el mundo simulado.

**Tests**: `test/` tiene un test por archivo de lógica y por launch.

### `ros2_ws/src/lawnbot_localization/` — dónde está el robot

- `launch/localization.launch.py`: arranca el modelo del robot (URDF), los dos filtros EKF y la conversión del GPS.
- `config/ekf_local.yaml`: filtro local (odometría + IMU), 15 Hz. Da la posición suave en `odom`.
- `config/ekf_global.yaml`: filtro global (suma el GPS), 10 Hz. Da la posición en `map`.
- `config/navsat.yaml`: convierte el GPS (lat/lon) a metros en el mapa.
- `test/`: verifica las frecuencias y tiempos de espera de los filtros.

### `ros2_ws/src/lawnbot_vision/` — cámara

- `lawnbot_vision/vision_node.py`: el nodo: toma imágenes de la cámara a 1 Hz y publica si hay pasto
  (`/vision/grass_present`) y los obstáculos (`/vision/obstacles`). Si no hay cámara, dice "sin pasto".
- `lawnbot_vision/detector.py`: carga el modelo con ncnn y abre la cámara (por su nombre fijo).
- `lawnbot_vision/seg_postprocess.py`: interpreta la salida del modelo (cajas, máscaras, punto de apoyo).
- `lawnbot_vision/vision_logic.py`: de la segmentación a "hay pasto" y a la distancia de cada obstáculo.
- `lawnbot_vision/calibrate.py`: calibración de distancia de la cámara (`ros2 run lawnbot_vision calibrar`).
- `config/vision.yaml`: modelo, cámara, frecuencia y calibración.
- `launch/vision.launch.py`: arranca solo la visión (normalmente se usa `nav.launch.py vision:=true`).
- `test/`: tests de la lógica, del post-proceso, de la calibración y de la reconexión de la cámara.

### `ros2_ws/src/lawnbot_hardware/` — motores de corte y TF-mini

- `launch/hardware.launch.py`: arranca los dos nodos de abajo.
- `lawnbot_hardware/cutting_motors_node.py`: maneja los motores de corte (puente BTS7960). Si deja de recibir
  órdenes, los apaga.
- `lawnbot_hardware/tfmini_node.py`: lee el sensor de distancia TF-mini Plus.

### `ros2_ws/src/berryimu_gps/` — IMU y GPS

- `launch/berryimu_gps.launch.py`: arranca el IMU y el GPS.
- `berryimu_gps/imu_node.py`: lee el BerryIMU y publica `/imu/data`.
- `berryimu_gps/gps_node.py`: lee el GPS (a través de `gpsd`) y publica `/gps/fix`.
- `berryimu_gps/gps_filter.py`: descarta los fixes malos del GPS (2D, error alto, saltos) y calcula su covarianza.
- `config/gps.yaml`: límites de ese filtro (error máximo, HDOP, velocidad máxima).
- `berryimu_gps/calibrate_accel.py`, `calibrate_gyro.py`, `calibrate_mag.py`: calibración del acelerómetro,
  giroscopio y magnetómetro.
- `config/imu_calibration.yaml`: resultados de esa calibración.
- `test/`: tests del IMU y el GPS.

### `vision/` — modelos y scripts sueltos

- `models/ncnn_320/`: **el modelo que usa el robot** (YOLOv8n-seg, 320 px, 7 clases). `metadata.yaml` tiene las
  clases.
- `models/ncnn_640/`: el mismo modelo a 640 px. **(no se usa en la Pi: es más lento)**.
- `pi_scripts/pi_infer.py`, `pi_stream.py`, `vision_core.py`, `potential_field.py`: pruebas de visión antes de
  ROS. **(viejo / no se usa: reemplazados por `lawnbot_vision` y `lawnbot_nav/potential_field.py`)**.

## Por dónde empezar

Para entender cómo se mueve el robot y cómo se comunican las partes, leer en este orden:

1. `docs/setup_navegacion.md`: la explicación general de la navegación, los topics y cómo probar.
2. `bridge/bridge.py`: qué manda la app y cómo entra a ROS (y qué se le devuelve a la app).
3. `ros2_ws/src/lawnbot_nav/launch/nav.launch.py`: qué nodos corren y cómo se conectan.
4. `ros2_ws/src/lawnbot_nav/lawnbot_nav/mode_manager.py` y `config/twist_mux.yaml`: cómo se elige el modo y quién
   manda el movimiento.
5. `ros2_ws/src/lawnbot_nav/lawnbot_nav/safety_logic.py`: las reglas de seguridad antes de mover las ruedas.
6. `firmware/MOW_TASKS.md` y `firmware/src/main_tasks.cpp`: qué hace el ESP32 con `/cmd_vel` y qué sensores
   publica.
