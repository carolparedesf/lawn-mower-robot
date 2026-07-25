# Documentación Técnica — RobotControl App

Documentación interna de la aplicación Flutter para control del robot móvil. Describe cada capa de la arquitectura, cómo fluyen los datos y qué hace exactamente cada archivo.

---

## Tabla de contenidos

1. [Visión general de la arquitectura](#1-visión-general-de-la-arquitectura)
2. [Árbol de archivos](#2-árbol-de-archivos)
3. [Capa de configuración](#3-capa-de-configuración)
4. [Capa de modelos](#4-capa-de-modelos)
5. [Capa de servicios — MqttService](#5-capa-de-servicios--mqttservice)
6. [Capa de providers — gestión de estado](#6-capa-de-providers--gestión-de-estado)
7. [Capa de pantallas](#7-capa-de-pantallas)
8. [Capa de widgets](#8-capa-de-widgets)
9. [Punto de entrada — main.dart](#9-punto-de-entrada--maindart)
10. [Flujo de datos completo](#10-flujo-de-datos-completo)
11. [Gestión del ciclo de vida](#11-gestión-del-ciclo-de-vida)
12. [Decisiones de diseño](#12-decisiones-de-diseño)

---

## 1. Visión general de la arquitectura

La app sigue una arquitectura en capas donde cada capa tiene una responsabilidad única y solo se comunica con la capa inmediatamente inferior:

```
┌──────────────────────────────────────────────────────┐
│                  UI  (Screens + Widgets)              │  Lee estado, llama métodos
├──────────────────────────────────────────────────────┤
│              Providers  (Estado reactivo)             │  ChangeNotifier + Provider
├──────────────────────────────────────────────────────┤
│              Services  (MqttService singleton)        │  Streams de datos crudos
├──────────────────────────────────────────────────────┤
│              Models  (estructuras de datos)           │  fromJson / toJson
├──────────────────────────────────────────────────────┤
│              Config  (constantes)                     │  IPs, topics, ports
└──────────────────────────────────────────────────────┘
```

**Principios aplicados:**

- **Única fuente de verdad:** cada dato vive en un solo lugar. La posición del robot vive en `RobotStateProvider`; los puntos del geofence viven en `GeofenceProvider`. La UI nunca guarda estado propio de negocio.
- **Flujo unidireccional:** los datos fluyen hacia abajo (providers → widgets). Las acciones del usuario fluyen hacia arriba (widget llama método del provider, que llama al servicio).
- **Singleton de servicio:** `MqttService` existe una única instancia durante toda la vida de la app, compartida por todos los providers.

---

## 2. Árbol de archivos

```
lib/
├── main.dart                         # Punto de entrada, tema y registro de providers
├── config/
│   └── mqtt_config.dart              # Constantes: IP, puerto, nombres de topics
├── models/
│   ├── robot_position.dart           # Datos GPS del robot
│   ├── robot_command.dart            # Estructura de un comando de movimiento
│   ├── robot_status.dart             # Estado operativo del robot
│   └── geofence.dart                 # Polígono geofence con validación
├── services/
│   └── mqtt_service.dart             # Singleton MQTT: conexión, pub/sub, streams
├── providers/
│   ├── connection_provider.dart      # Estado de la conexión + cambio de modo
│   ├── robot_state_provider.dart     # Posición actual, historial, status
│   └── geofence_provider.dart        # Puntos del polígono, add/undo/clear/send
├── screens/
│   ├── home_screen.dart              # Scaffold raíz con BottomNavigationBar
│   ├── control_screen.dart           # Tab de control manual
│   └── map_screen.dart               # Tab de mapa con geofence
└── widgets/
    ├── joystick_widget.dart           # Joystick analógico con throttle
    ├── connection_indicator.dart      # Chip verde/naranja/rojo de conexión
    ├── mode_selector.dart             # Botones Manual / Autónomo / Stop
    ├── status_bar.dart                # Barra de estado (modo, batería, alerta)
    └── geofence_toolbar.dart          # Toolbar: undo, clear, send
```

---

## 3. Capa de configuración

### `config/mqtt_config.dart`

Clase con exclusivamente constantes estáticas. Ningún otro archivo hardcodea valores de red.

```dart
class MqttConfig {
  static const String broker    = '192.168.1.100'; // IP del Raspberry Pi
  static const int    port      = 1883;
  static const String clientId  = 'robotcontrol_app';

  // Topics de salida (App → Robot)
  static const String topicCmdMove     = 'robot/cmd/move';
  static const String topicCmdMode     = 'robot/cmd/mode';
  static const String topicCmdGeofence = 'robot/cmd/geofence';

  // Topics de entrada (Robot → App)
  static const String topicPosition = 'robot/position';
  static const String topicStatus   = 'robot/status';
}
```

**Por qué existe esta clase:** centralizar la IP y los topics permite cambiar el broker o renombrar un topic en un único lugar sin buscar en toda la base de código.

---

## 4. Capa de modelos

Los modelos son clases de datos puras (sin lógica de negocio ni dependencias de Flutter). Todos implementan `fromJson` y `toJson` para la serialización MQTT.

### `models/robot_position.dart`

Representa una lectura GPS del robot.

| Campo | Tipo | Origen JSON |
|---|---|---|
| `lat` | `double` | `"lat"` |
| `lng` | `double` | `"lng"` |
| `heading` | `double` | `"hdg"` (opcional, default 0.0) |
| `timestamp` | `DateTime` | Generado localmente al recibir |

El `timestamp` no viene del robot — se genera en el momento de deserialización con `DateTime.now()`. Esto es intencional: el robot no tiene por qué sincronizar relojes con la app.

---

### `models/robot_status.dart`

Estado operativo del robot. Incluye el getter de conveniencia `hasGeofenceBreach` para evitar que la UI compare strings directamente.

```dart
bool get hasGeofenceBreach => alert == 'geofence_breach';
```

Los valores posibles de `mode` son `idle`, `manual`, `auto`, `stop`. Los valores posibles de `alert` son `none` y `geofence_breach`.

---

### `models/robot_command.dart`

Estructura de un comando de movimiento. No se usa para recibir datos del robot (el robot no envía comandos a la app), sino como modelo de referencia para documentar el formato. La serialización real la hace `JoystickWidget` directamente con `jsonEncode`.

---

### `models/geofence.dart`

Contiene la lista de puntos y la validación mínima de 3 vértices:

```dart
class Geofence {
  final List<LatLng> points;
  bool get isValid => points.length >= 3;

  Map<String, dynamic> toJson() => {
    'points': points.map((p) => {'lat': p.latitude, 'lng': p.longitude}).toList(),
  };
}
```

El polígono se considera cerrado implícitamente: el robot conecta el último punto con el primero para calcular si está dentro del área. La app no duplica el primer punto al final del array.

---

## 5. Capa de servicios — MqttService

### `services/mqtt_service.dart`

El núcleo de comunicación. Es el único archivo que interactúa directamente con el paquete `mqtt_client`.

#### Patrón Singleton

```dart
class MqttService {
  MqttService._();                              // constructor privado
  static final MqttService instance = MqttService._(); // instancia única
}
```

Garantiza que solo exista una conexión MQTT durante toda la vida de la app. Los providers acceden a través de `MqttService.instance`.

#### Streams de broadcast

```dart
final _positionController   = StreamController<RobotPosition>.broadcast();
final _statusController     = StreamController<RobotStatus>.broadcast();
final _connectionController = StreamController<bool>.broadcast();
```

Se usan `broadcast` (y no streams normales de un solo escucha) porque tanto los providers como potencialmente múltiples widgets pueden necesitar escuchar el mismo stream simultáneamente. Un stream normal solo permite un listener; un broadcast stream permite N listeners.

#### Proceso de conexión (`connect`)

```
1. Crear MqttServerClient con IP y clientId
2. Configurar keepAlive = 20 segundos
3. Registrar callbacks onConnected / onDisconnected
4. Construir MqttConnectMessage con startClean() → sesión limpia sin mensajes retenidos
5. Llamar _client.connect() → async, puede lanzar excepción si no hay red
6. Verificar que connectionStatus.state == connected
7. Suscribirse a robot/position (QoS 0) y robot/status (QoS 1)
8. Registrar listener en _client.updates para procesar mensajes entrantes
```

Si la conexión falla (excepción o estado incorrecto), el método retorna `false` y `ConnectionProvider` lo refleja en la UI.

#### Procesamiento de mensajes entrantes (`_onMessage`)

```dart
void _onMessage(List<MqttReceivedMessage<MqttMessage>> messages) {
  for (final msg in messages) {
    final payload = MqttPublishPayload.bytesToStringAsString(
        (msg.payload as MqttPublishMessage).payload.message);
    final json = jsonDecode(payload) as Map<String, dynamic>;

    if (msg.topic == MqttConfig.topicPosition) {
      _positionController.add(RobotPosition.fromJson(json));
    } else if (msg.topic == MqttConfig.topicStatus) {
      _statusController.add(RobotStatus.fromJson(json));
    }
  }
}
```

El bloque `try/catch` vacío descarta silenciosamente mensajes malformados. Si el robot envía JSON inválido, la app simplemente ignora ese mensaje sin crashear.

#### Publicación (`publish`)

```dart
void publish(String topic, String payload, {MqttQos qos = MqttQos.atMostOnce}) {
  if (!_connected) return;                          // guard: no enviar si no hay conexión
  final builder = MqttClientPayloadBuilder()..addString(payload);
  _client.publishMessage(topic, qos, builder.payload!);
}
```

El guard `if (!_connected) return` protege contra llamadas cuando la conexión se perdió pero los providers aún no actualizaron su estado.

---

## 6. Capa de providers — gestión de estado

Los providers son `ChangeNotifier` registrados en el árbol de widgets con `MultiProvider` en `main.dart`. Cuando llaman `notifyListeners()`, todos los `Consumer` o `context.watch` que dependen de ellos se reconstruyen.

### `providers/connection_provider.dart`

Responsabilidades:
- Mantener el estado de la conexión (`_connected`, `_connecting`)
- Almacenar la IP del broker actual (`_broker`)
- Exponer `connect()` y `disconnect()`
- Exponer `setMode()` para publicar cambios de modo al robot

#### Escucha del stream de conexión

```dart
ConnectionProvider() {
  _sub = MqttService.instance.connectionStream.listen((connected) {
    _connected = connected;
    notifyListeners(); // reconstruye ConnectionIndicator, ModeSelector, JoystickWidget
  });
}
```

El provider se suscribe al `connectionStream` de `MqttService` en su constructor. Así recibe el evento de desconexión incluso si el broker se cae de forma inesperada (sin que el usuario lo solicite).

#### Flujo de `connect()`

```
1. Guard: si ya está conectando, no hacer nada (evita doble tap)
2. Actualizar _broker si se pasó una IP nueva
3. _connecting = true → notifyListeners() → UI muestra "Conectando..."
4. Llamar MqttService.instance.connect() → espera resultado
5. _connected = resultado, _connecting = false → notifyListeners() → UI actualiza
```

#### `setMode()`

Serializa el modo a JSON y lo publica con QoS 1 (garantizado). No modifica ningún estado local — el estado del modo viene del robot vía `robot/status` y lo gestiona `RobotStateProvider`.

---

### `providers/robot_state_provider.dart`

Responsabilidades:
- Almacenar la última posición conocida del robot (`_currentPosition`)
- Mantener el historial de posiciones para dibujar la polyline (máximo 200 puntos)
- Almacenar el último status del robot (`_currentStatus`)

#### Historial de posiciones

```dart
const int _maxHistory = 200;

void _onPosition(RobotPosition pos) {
  _currentPosition = pos;
  _positionHistory.add(LatLng(pos.lat, pos.lng));
  if (_positionHistory.length > _maxHistory) {
    _positionHistory.removeAt(0);   // descarta el más antiguo (FIFO)
  }
  notifyListeners();
}
```

El límite de 200 posiciones es un balance entre mostrar suficiente recorrido y no acumular memoria indefinidamente. A 1 mensaje/segundo, 200 puntos representan ~3 minutos de trayectoria.

El getter expone una lista inmutable:
```dart
List<LatLng> get positionHistory => List.unmodifiable(_positionHistory);
```

Esto previene que la UI modifique la lista directamente, lo que rompería el patrón.

---

### `providers/geofence_provider.dart`

Responsabilidades:
- Mantener la lista de puntos del polígono en construcción
- Exponer operaciones: `addPoint`, `undo`, `clear`, `sendGeofence`
- Validar antes de enviar (mínimo 3 puntos)

#### `sendGeofence()`

```dart
bool sendGeofence() {
  final fence = Geofence(points: List.from(_points)); // copia defensiva
  if (!fence.isValid) return false;

  final payload = jsonEncode(fence.toJson());
  MqttService.instance.publish(
    MqttConfig.topicCmdGeofence,
    payload,
    qos: MqttQos.atLeastOnce,
  );
  return true;
}
```

Retorna `bool` para que la UI pueda mostrar un `SnackBar` diferente según si el envío fue exitoso o no. No limpia la lista tras el envío, permitiendo al usuario ajustar el geofence y reenviarlo.

---

## 7. Capa de pantallas

### `screens/home_screen.dart`

Scaffold raíz de la app. Contiene:

- **AppBar** con título "RobotControl", `ConnectionIndicator` y botón de configuración
- **BottomNavigationBar** con dos tabs: Control (índice 0) y Mapa (índice 1)
- **Cuerpo:** un arreglo estático `_screens` indexado por `_currentIndex`

```dart
static const _screens = [
  ControlScreen(),
  MapScreen(),
];
```

Las pantallas son `const` porque no reciben parámetros — todo su estado proviene de los providers. Esto permite a Flutter reutilizar las instancias sin reconstruirlas al cambiar de tab.

#### Diálogo de configuración de IP

Al tocar el ícono de configuración, se abre un `AlertDialog` con un `TextField` pre-poblado con la IP actual del broker. Al confirmar, llama `ConnectionProvider.connect(broker: nuevaIp)`.

---

### `screens/control_screen.dart`

Pantalla sin estado propio (`StatelessWidget`). Layout vertical:

```
StatusBar        ← información del robot (modo, batería, alerta)
─────────────
[espacio]
  "Control Manual"
  JoystickWidget
  ModeSelector
[espacio]
```

Es un `StatelessWidget` porque todo su contenido reactivo vive en los providers. No necesita `setState`.

---

### `screens/map_screen.dart`

La pantalla más compleja. Es un `StatefulWidget` porque tiene estado local de UI (modo geofence activo, modo seguir robot) que no necesita ser compartido con otras pantallas.

#### Estado local

```dart
bool _geofenceMode = false;  // true: tap en mapa agrega puntos
bool _followRobot  = true;   // true: mapa se mueve con el robot
```

Estos estados son específicos de esta pantalla (no son estado de negocio), por eso viven en `State` y no en un provider.

#### Capas del mapa (orden de renderizado, de abajo hacia arriba)

```
1. TileLayer        → tiles OSM de internet
2. PolylineLayer    → traza de recorrido (azul semitransparente)
3. PolygonLayer     → polígono geofence (verde o rojo si breach)
4. MarkerLayer      → puntos del geofence + marcador del robot
```

Cada capa es condicional:
- `PolylineLayer` solo se dibuja si hay más de 1 punto en el historial
- `PolygonLayer` solo se dibuja si hay al menos 2 puntos geofence
- El marcador del robot solo se dibuja si `currentPosition != null`

#### Rotación del marcador del robot

```dart
Transform.rotate(
  angle: robotPos.heading * math.pi / 180, // grados → radianes
  child: const Icon(Icons.navigation, color: Colors.blue, size: 32),
)
```

El ícono `Icons.navigation` apunta hacia arriba por defecto (Norte). Rotarlo con el `heading` del robot hace que apunte en la dirección real de movimiento.

#### Modo seguir robot

```dart
if (_followRobot && robotPos != null) {
  WidgetsBinding.instance.addPostFrameCallback((_) {
    _mapController.move(center, _mapController.camera.zoom);
  });
}
```

El `addPostFrameCallback` es necesario porque no se puede modificar el `MapController` durante el `build`. Se programa para ejecutar después de que el frame actual esté renderizado.

Cuando el usuario hace pan manual en el mapa, se desactiva el seguimiento automático:
```dart
onPositionChanged: (_, hasGesture) {
  if (hasGesture && _followRobot) {
    setState(() => _followRobot = false);
  }
},
```

#### Modo geofence

Con `_geofenceMode = true`, el callback `onTap` del mapa agrega puntos al `GeofenceProvider`:
```dart
onTap: _geofenceMode
    ? (_, latlng) => geofence.addPoint(latlng)
    : null,
```

Cuando es `null`, el mapa vuelve a su comportamiento por defecto (pan/zoom).

---

## 8. Capa de widgets

### `widgets/joystick_widget.dart`

El widget más técnicamente complejo por el throttle.

#### Por qué se necesita throttle

El `flutter_joystick` dispara el callback `listener` en cada frame de la animación del stick (~60 veces por segundo). Sin throttle, se publicarían 60 mensajes MQTT por segundo, saturando el broker y la red WiFi.

#### Implementación del throttle

```dart
Timer? _throttle;

void _onMove(StickDragDetails details) {
  // 1. Calcular dirección y velocidad
  _lastDir   = dir;
  _lastSpeed = speed;

  // 2. Si ya hay un timer activo, salir (descartar este evento)
  if (_throttle != null) return;

  // 3. Programar publicación en 50ms
  _throttle = Timer(const Duration(milliseconds: 50), () {
    _throttle = null;  // limpiar para permitir el siguiente ciclo
    _publish();        // publicar el estado MÁS RECIENTE (no el primero)
  });
}
```

Este patrón se llama **trailing throttle**: espera 50ms, luego publica el valor más reciente capturado durante ese intervalo. Resultado: máximo 20 publicaciones/segundo (20 Hz).

#### Conversión de coordenadas XY → dirección discreta

```dart
if (x.abs() < 0.2 && y.abs() < 0.2) {
  dir = 'stop';                         // zona muerta central
} else if (y.abs() >= x.abs()) {
  dir = y < 0 ? 'fwd' : 'back';        // componente vertical dominante
} else {
  dir = x > 0 ? 'right' : 'left';      // componente horizontal dominante
}
speed = dominantAxis.abs().clamp(0.0, 1.0);
```

La zona muerta del 20% evita que el joystick envíe movimiento cuando está en reposo pero no exactamente centrado (drift del dedo).

#### Deshabilitación visual

Cuando no hay conexión, el joystick se muestra semitransparente y no responde a toques:
```dart
Opacity(opacity: enabled ? 1.0 : 0.4,
  child: IgnorePointer(ignoring: !enabled,
    child: Joystick(...)
  )
)
```

`IgnorePointer` bloquea los eventos de toque sin remover el widget del árbol, manteniendo el layout estable.

---

### `widgets/connection_indicator.dart`

Chip de tres estados que refleja `ConnectionProvider`:

| Estado | Color | Texto |
|---|---|---|
| `isConnected == true` | Verde | "Conectado" |
| `isConnecting == true` | Naranja | "Conectando..." |
| Ninguno | Rojo | "Desconectado" |

Usa `context.watch<ConnectionProvider>()` para reconstruirse automáticamente en cada cambio de estado.

---

### `widgets/mode_selector.dart`

Tres `ElevatedButton` que se resaltan cuando el modo activo coincide con el suyo:

```dart
backgroundColor: isActive ? color : null,
foregroundColor: isActive ? Colors.white : null,
```

El botón "Stop" usa `Colors.red` como color activo en lugar del color primario del tema.

Los botones se deshabilitan (`onPressed: null`) cuando no hay conexión MQTT, lo que Flutter renderiza automáticamente con opacidad reducida.

---

### `widgets/status_bar.dart`

Barra informativa en la parte superior de `ControlScreen`. Muestra:
- **Modo actual** del robot (viene de `RobotStatus.mode`)
- **Batería** con ícono que cambia a `battery_alert` y color rojo cuando cae por debajo del 20%
- **Alerta geofence** (solo visible si `hasGeofenceBreach == true`)

El fondo cambia a `Colors.red.shade100` cuando hay breach de geofence, dando señal visual inmediata aunque el usuario no esté mirando el texto.

---

### `widgets/geofence_toolbar.dart`

Toolbar compacta que aparece en la parte inferior del mapa cuando el modo geofence está activo. Muestra el contador de puntos y tres botones de acción:

| Botón | Acción | Habilitado cuando |
|---|---|---|
| Undo | `GeofenceProvider.undo()` | Hay al menos 1 punto |
| Clear | `GeofenceProvider.clear()` | Hay al menos 1 punto |
| Send | `GeofenceProvider.sendGeofence()` | Hay al menos 3 puntos |

El botón Send muestra un `SnackBar` confirmando el resultado:
```dart
final ok = geofence.sendGeofence();
ScaffoldMessenger.of(context).showSnackBar(
  SnackBar(content: Text(ok ? 'Geofence enviado' : 'Mínimo 3 puntos'))
);
```

---

## 9. Punto de entrada — main.dart

```dart
void main() {
  runApp(const RobotControlApp());
}
```

`RobotControlApp` es un `StatelessWidget` que construye el `MaterialApp` envuelto en `MultiProvider`:

```dart
MultiProvider(
  providers: [
    ChangeNotifierProvider(create: (_) => ConnectionProvider()),
    ChangeNotifierProvider(create: (_) => RobotStateProvider()),
    ChangeNotifierProvider(create: (_) => GeofenceProvider()),
  ],
  child: MaterialApp(...)
)
```

Los tres providers se crean aquí (en la raíz del árbol) para que cualquier widget en cualquier pantalla pueda acceder a ellos con `context.watch` o `context.read` sin recibir los objetos por constructor.

El tema define el color primario rosa `#E91E8C` con `ColorScheme.fromSeed`, que genera automáticamente variantes de color (surface, on-primary, secondary, etc.) para todos los componentes Material 3.

---

## 10. Flujo de datos completo

### Datos entrantes (Robot → App)

```
Broker MQTT
    │
    │  publica en robot/position o robot/status
    ▼
MqttService._onMessage()
    │  deserializa JSON → RobotPosition o RobotStatus
    │  agrega al StreamController correspondiente
    ▼
RobotStateProvider._onPosition() / _onStatus()
    │  actualiza _currentPosition / _positionHistory / _currentStatus
    │  llama notifyListeners()
    ▼
Widgets que tienen context.watch<RobotStateProvider>()
    │  se reconstruyen automáticamente
    ▼
MapScreen → mueve marcador, extiende polyline, colorea polígono
StatusBar → actualiza modo, batería, alerta
```

### Datos salientes (App → Robot) — Joystick

```
Usuario mueve el joystick
    │
    ▼
JoystickWidget._onMove()
    │  calcula dir y speed desde coordenadas XY
    │  actualiza _lastDir, _lastSpeed
    │  si no hay throttle activo: crea Timer(50ms)
    ▼
Timer expira después de 50ms
    │
    ▼
JoystickWidget._publish()
    │  jsonEncode({'dir': ..., 'speed': ...})
    │  MqttService.instance.publish(topicCmdMove, payload, QoS 0)
    ▼
Broker MQTT → Robot
```

### Datos salientes — Cambio de modo

```
Usuario toca botón Manual / Autónomo / Stop
    │
    ▼
ModeSelector → ConnectionProvider.setMode('manual')
    │  jsonEncode({'mode': 'manual'})
    │  MqttService.publish(topicCmdMode, payload, QoS 1)
    ▼
Broker MQTT → Robot
    │  robot cambia su modo interno
    │  robot publica robot/status con el nuevo modo
    ▼
MqttService → RobotStateProvider → StatusBar / ModeSelector (actualiza highlight)
```

### Datos salientes — Geofence

```
Usuario activa modo geofence en MapScreen
Usuario hace tap en el mapa → MapOptions.onTap
    │
    ▼
GeofenceProvider.addPoint(latlng)
    │  agrega punto a _points
    │  notifyListeners()
    ▼
MapScreen reconstruye PolygonLayer y MarkerLayer con los nuevos puntos

[cuando el usuario presiona "Enviar" en GeofenceToolbar]
    │
    ▼
GeofenceProvider.sendGeofence()
    │  valida >= 3 puntos
    │  serializa a JSON
    │  MqttService.publish(topicCmdGeofence, payload, QoS 1)
    ▼
Broker MQTT → Robot
```

---

## 11. Gestión del ciclo de vida

### Suscripciones a streams

Cada provider que escucha un stream guarda la `StreamSubscription` y la cancela en `dispose()`:

```dart
StreamSubscription<RobotPosition>? _posSub;

RobotStateProvider() {
  _posSub = MqttService.instance.positionStream.listen(_onPosition);
}

@override
void dispose() {
  _posSub?.cancel();   // evita llamar notifyListeners en un provider ya destruido
  super.dispose();
}
```

Sin cancelar la suscripción, el callback seguiría ejecutándose tras destruir el provider, causando el error `"A listener was called after it was disposed"`.

### Timer del joystick

```dart
@override
void dispose() {
  _throttle?.cancel();  // cancela el timer pendiente si el widget se destruye
  super.dispose();
}
```

Sin cancelar el timer, podría dispararse después de que el `State` fue destruido e intentar usar `context`, causando un crash.

### MqttService

No tiene `dispose` automático vinculado al árbol de widgets (es un singleton que vive durante toda la app). Sus `StreamController` se cierran con `MqttService.instance.dispose()`, que debería llamarse al cerrar la app completamente.

---

## 12. Decisiones de diseño

### ¿Por qué Provider y no Riverpod o Bloc?

Provider es la solución oficial recomendada por el equipo de Flutter para proyectos académicos y de complejidad media. Riverpod y Bloc tienen más boilerplate y conceptos adicionales que no se justifican para esta app.

### ¿Por qué singleton para MqttService?

Una conexión MQTT es un recurso escaso y con estado (socket TCP). Tener múltiples instancias crearía múltiples conexiones al broker, lo que rompería el protocolo MQTT (el broker desconecta la sesión anterior cuando llega una nueva con el mismo `clientId`). El singleton garantiza exactamente una conexión.

### ¿Por qué streams en MqttService en lugar de callbacks?

Los streams de broadcast desacoplan completamente el servicio de sus consumidores. `MqttService` no necesita saber que existen `RobotStateProvider` o `ConnectionProvider`. Cualquier futuro provider puede suscribirse sin modificar `MqttService`.

### ¿Por qué el historial de posiciones vive en el provider y no en el widget?

Si viviera en `MapScreen._MapScreenState`, se perdería cada vez que el usuario cambia al tab de Control y vuelve al Mapa, porque Flutter destruye y recrea el widget. Al vivir en `RobotStateProvider` (que vive en la raíz del árbol), el historial persiste durante toda la sesión.

### ¿Por qué `_geofenceMode` y `_followRobot` son estado local y no están en un provider?

Son estados de UI puros, específicos de la pantalla de mapa, sin valor para otras partes de la app. El criterio para usar un provider es: "¿otro widget en otra pantalla necesita este dato?". Si la respuesta es no, el `State` local es la opción correcta.

### QoS 0 para movimiento, QoS 1 para modo y geofence

- `robot/cmd/move` se publica a 20 Hz. Perder uno o dos mensajes es irrelevante porque llega el siguiente en 50ms. QoS 1 a esta frecuencia introduciría ACKs que saturarían la red.
- `robot/cmd/mode` y `robot/cmd/geofence` se publican raramente pero deben llegar. QoS 1 garantiza entrega con reintento automático.
