# CLAUDE.md — RoboControl App

## Proyecto

App móvil Flutter para control de robot móvil basado en Raspberry Pi. Comunicación vía MQTT sobre WiFi local. El robot usa motores DC (driver L298N), GPS (NEO-6M) y corre un script Python con `gpiozero`, `paho-mqtt` y `gpsd`.

## Stack

- **Framework:** Flutter 3.x (Dart)
- **State Management:** Provider (simplicidad académica)
- **Mapa:** `flutter_map` + `latlong2` (OpenStreetMap, sin API key)
- **Comunicación:** `mqtt_client` (paquete Dart)
- **Serialización:** `dart:convert` (JSON)
- **Joystick:** `flutter_joystick` o widget custom con `GestureDetector`
- **Target:** Android (mínimo SDK 21)

## Arquitectura

```
lib/
├── main.dart                         # MaterialApp, tema, navegación
├── config/
│   └── mqtt_config.dart              # host, port, topics (constantes)
├── services/
│   └── mqtt_service.dart             # singleton: connect, disconnect, publish, subscribe, stream
├── models/
│   ├── robot_position.dart           # lat, lng, heading, timestamp — fromJson/toJson
│   ├── robot_command.dart            # direction, speed, mode — fromJson/toJson
│   ├── robot_status.dart             # mode, battery, alert — fromJson/toJson
│   └── geofence.dart                 # List<LatLng>, validación mín 3 puntos, toJson
├── providers/
│   ├── connection_provider.dart      # ChangeNotifier: estado conexión MQTT
│   ├── robot_state_provider.dart     # ChangeNotifier: posición, modo, batería (se alimenta de MqttService streams)
│   └── geofence_provider.dart        # ChangeNotifier: puntos del polígono, add/undo/clear/send
├── screens/
│   ├── home_screen.dart              # BottomNavigationBar con 2 tabs: Control y Mapa
│   ├── control_screen.dart           # Joystick + botones modo (manual/auto/stop) + status
│   └── map_screen.dart               # Mapa con marcador robot + polígono geofence + toolbar
└── widgets/
    ├── joystick_widget.dart           # Pad analógico, emite direction + speed
    ├── connection_indicator.dart      # Chip verde/rojo estado MQTT
    ├── mode_selector.dart             # Botones Manual / Autónomo / Stop
    ├── status_bar.dart                # Modo actual, batería, alerta geofence
    └── geofence_toolbar.dart          # Botones: añadir punto, deshacer, limpiar, enviar
```

## Topics MQTT

| Topic | Dirección | Payload | QoS |
|---|---|---|---|
| `robot/cmd/move` | App → Robot | `{"dir":"fwd\|back\|left\|right\|stop","speed":0.0-1.0}` | 0 |
| `robot/cmd/mode` | App → Robot | `{"mode":"manual\|auto\|stop"}` | 1 |
| `robot/cmd/geofence` | App → Robot | `{"points":[{"lat":float,"lng":float},...]}`| 1 |
| `robot/position` | Robot → App | `{"lat":float,"lng":float,"hdg":float}` | 0 |
| `robot/status` | Robot → App | `{"mode":"idle\|manual\|auto","battery":int,"alert":"none\|geofence_breach"}` | 1 |

- QoS 0 para comandos de movimiento (alta frecuencia, tolera pérdida).
- QoS 1 para cambios de modo y geofence (deben llegar).

## Convenciones de Código

- **Idioma código:** inglés (variables, clases, métodos, comentarios).
- **Nombrado:** `snake_case` para archivos, `camelCase` para variables/métodos, `PascalCase` para clases.
- **Modelos:** siempre implementar `factory fromJson(Map<String, dynamic>)` y `Map<String, dynamic> toJson()`.
- **MqttService:** singleton con `StreamController` por cada topic de entrada. Los providers escuchan estos streams.
- **No hardcodear** host/port del broker. Todo en `mqtt_config.dart`.
- **Joystick:** emitir comandos con throttle de 50ms (20 Hz máx) para no saturar el broker.
- **Geofence:** validar mínimo 3 puntos antes de permitir envío. El polígono se cierra automáticamente (último punto conecta con el primero).
- **Dispose:** siempre cancelar suscripciones y cerrar streams en `dispose()`.

## Dependencias (pubspec.yaml)

```yaml
dependencies:
  flutter:
    sdk: flutter
  flutter_map: ^6.0.0
  latlong2: ^0.9.0
  mqtt_client: ^10.0.0
  provider: ^6.0.0
  flutter_joystick: ^0.1.0

dev_dependencies:
  flutter_test:
    sdk: flutter
  flutter_lints: ^3.0.0
```

## Configuración MQTT por defecto

```dart
// config/mqtt_config.dart
class MqttConfig {
  static const String broker = '192.168.1.100'; // IP del Raspberry Pi en la red local
  static const int port = 1883;
  static const String clientId = 'robocontrol_app';

  // Topics de salida (App → Robot)
  static const String topicCmdMove = 'robot/cmd/move';
  static const String topicCmdMode = 'robot/cmd/mode';
  static const String topicCmdGeofence = 'robot/cmd/geofence';

  // Topics de entrada (Robot → App)
  static const String topicPosition = 'robot/position';
  static const String topicStatus = 'robot/status';
}
```

## Flujo de Datos

```
MqttService (singleton)
    │
    ├── Stream<RobotPosition> positionStream  ◄── robot/position
    ├── Stream<RobotStatus> statusStream      ◄── robot/status
    │
    ▼
RobotStateProvider (ChangeNotifier)
    │  escucha ambos streams
    │  expone: currentPosition, currentStatus, positionHistory (para polyline)
    ▼
UI (Consumer<RobotStateProvider>)
    │  rebuilds en cada cambio
    ▼
MapScreen / ControlScreen / StatusBar
```

```
UI (botones, joystick)
    │
    │  llama métodos de providers
    ▼
GeofenceProvider.sendGeofence()
ConnectionProvider.setMode("auto")
    │
    │  internamente llaman MqttService.publish(topic, payload)
    ▼
MqttService → Broker → Robot
```

## Comportamiento del Mapa

- Centrado inicial: posición del robot al recibir primer mensaje, o fallback a Asunción (-25.2867, -57.6470).
- Marcador del robot: ícono rotado según `heading`.
- Polyline de recorrido: guardar últimas 200 posiciones, color semitransparente.
- Modo geofence: tap en mapa agrega `LatLng` a la lista. Se dibuja polígono con fill semitransparente y borde definido. Toolbar permite undo (quitar último punto), clear (borrar todo), send (publicar al robot).
- Cuando el polígono está definido y el robot reporta `alert: geofence_breach`, resaltar el borde en rojo.

## Testing y Simulación

Para probar sin hardware, correr el script simulador Python en la misma red:

```bash
# En cualquier PC con Python y Mosquitto instalado
python3 fake_robot.py
```

El simulador publica posiciones GPS fake moviéndose en círculo y responde a cambios de modo. Ver archivo `tools/fake_robot.py` en el repo.

## Errores Comunes

- **MqttService no conecta:** verificar que Mosquitto esté corriendo en el Pi (`sudo systemctl status mosquitto`) y que el celular esté en la misma red WiFi.
- **GPS sin señal:** el NEO-6M necesita cielo abierto y ~30s de cold start. Indoor no funciona.
- **Joystick lagea:** verificar que el throttle esté activo (50ms). Sin throttle se envían cientos de mensajes por segundo.
- **Mapa no carga tiles:** `flutter_map` necesita conexión a internet para descargar tiles OSM. En red local aislada, considerar tile server offline.
