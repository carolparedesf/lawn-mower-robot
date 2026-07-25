# RobotControl App

App móvil Flutter para control remoto de un robot móvil basado en Raspberry Pi. Permite control manual con joystick, cambio de modo de operación y definición de zonas geofence sobre un mapa OpenStreetMap. La comunicación entre la app y el robot se realiza vía MQTT sobre WiFi local.

---

## Tabla de contenidos

1. [Instalación del entorno de desarrollo (Ubuntu)](#1-instalación-del-entorno-de-desarrollo-ubuntu)
2. [Instalación de Flutter](#2-instalación-de-flutter)
3. [Instalación de Android Studio](#3-instalación-de-android-studio)
4. [Correr y depurar la app](#4-correr-y-depurar-la-app)
5. [Guía para el ingeniero electrónico — Construcción del robot](#5-guía-para-el-ingeniero-electrónico--construcción-del-robot)
6. [Protocolo MQTT — Mensajes de la app y del robot](#6-protocolo-mqtt--mensajes-de-la-app-y-del-robot)
7. [Cómo probar la app](#7-cómo-probar-la-app)

---

## 1. Instalación del entorno de desarrollo (Ubuntu)

### Requisitos previos

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y curl git unzip xz-utils zip libglu1-mesa wget clang cmake ninja-build pkg-config libgtk-3-dev
```

### Java (necesario para Android Studio y Flutter)

```bash
sudo apt install -y openjdk-17-jdk
java -version   # debe mostrar openjdk 17
```

---

## 2. Instalación de Flutter

### Opción A — con snap (más simple)

```bash
sudo snap install flutter --classic
flutter --version
```

### Opción B — instalación manual

```bash
# Descargar SDK (verificar última versión en https://docs.flutter.dev/release/archive)
cd ~
wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.24.0-stable.tar.xz
tar xf flutter_linux_3.24.0-stable.tar.xz

# Agregar al PATH (agregar al final de ~/.bashrc o ~/.zshrc)
echo 'export PATH="$HOME/flutter/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

flutter --version
```

### Verificar instalación

```bash
flutter doctor
```

El comando mostrará qué dependencias faltan. El objetivo es que `Android toolchain` y `Android Studio` aparezcan con tilde verde. Es normal que otras plataformas (Linux desktop, Chrome) no estén configuradas si solo se desarrolla para Android.

---

## 3. Instalación de Android Studio

### Descargar e instalar

```bash
# Instalar con snap
sudo snap install android-studio --classic

# O descargar manualmente desde https://developer.android.com/studio
# y extraer en /opt/android-studio
```

### Primera ejecución y configuración inicial

1. Ejecutar Android Studio: `android-studio` en terminal o desde el menú de aplicaciones.
2. Al abrirse por primera vez, seguir el asistente de instalación estándar.
3. Instalar el **Android SDK** cuando lo solicite (acepta las licencias).
4. En `SDK Manager` > `SDK Platforms`, instalar **Android 14 (API 34)** o superior.
5. En `SDK Manager` > `SDK Tools`, asegurarse de tener marcados:
   - Android SDK Build-Tools
   - Android SDK Platform-Tools
   - Android Emulator
   - Intel x86 Emulator Accelerator (HAXM) — si el procesador es Intel

### Aceptar licencias de Android SDK

```bash
flutter doctor --android-licenses
# Tipear 'y' a cada pregunta
```

### Instalar plugin de Flutter en Android Studio

1. Abrir Android Studio > `File` > `Settings` > `Plugins`
2. Buscar **Flutter** e instalar (instala Dart automáticamente)
3. Reiniciar Android Studio

---

## 4. Correr y depurar la app

### Clonar e instalar dependencias del proyecto

```bash
git clone <url-del-repositorio> robotmapapp
cd robotmapapp
flutter pub get
```

### Opción A — Con teléfono Android físico

#### Preparar el teléfono

1. En el teléfono, ir a `Ajustes` > `Acerca del teléfono`.
2. Tocar `Número de compilación` 7 veces hasta habilitar las **Opciones de desarrollador**.
3. Ir a `Ajustes` > `Sistema` > `Opciones de desarrollador`.
4. Activar **Depuración USB**.
5. Conectar el teléfono a la PC con un cable USB que soporte datos (no solo carga).
6. En el teléfono aparecerá un diálogo: `¿Permitir depuración USB desde esta PC?` — tocar **Permitir**.

#### Verificar que el teléfono es reconocido

```bash
flutter devices
# Debe aparecer el nombre del teléfono en la lista
```

#### Correr la app en el teléfono

```bash
flutter run
```

Para elegir un dispositivo específico si hay varios:

```bash
flutter run -d <device-id>
```

### Opción B — Con emulador Android

#### Crear un emulador (AVD) en Android Studio

1. Abrir Android Studio > `Device Manager` (ícono en la barra lateral derecha).
2. Hacer clic en `Create Device`.
3. Elegir un modelo (ej. Pixel 7) y hacer clic en `Next`.
4. Descargar y seleccionar una imagen del sistema (ej. **API 34, Google Play**).
5. Finalizar la creación y presionar el botón ▶ para iniciar el emulador.

#### Correr la app en el emulador

Con el emulador ya abierto:

```bash
flutter run
# Flutter detecta el emulador automáticamente
```

### Comandos útiles de depuración

| Comando | Descripción |
|---|---|
| `flutter run` | Corre la app en modo debug con hot reload |
| `r` (en consola) | Hot reload — recarga el código sin reiniciar |
| `R` (en consola) | Hot restart — reinicia la app completa |
| `q` (en consola) | Detiene la app |
| `flutter run --release` | Corre en modo release (sin debug, más rápido) |
| `flutter build apk` | Genera el APK en `build/app/outputs/apk/release/` |
| `flutter logs` | Muestra los logs del dispositivo en tiempo real |
| `flutter analyze` | Analiza errores estáticos en el código |

### Ver logs en tiempo real

```bash
# Con la app corriendo, los print() de Dart aparecen en la consola
# Para ver logs del sistema Android:
adb logcat | grep flutter
```

### Depuración con Android Studio

1. Abrir el proyecto en Android Studio (`File` > `Open` > seleccionar carpeta `robotmapapp`).
2. Seleccionar el dispositivo en el menú desplegable de la barra superior.
3. Presionar el botón 🐛 **Debug** (o `Shift+F9`).
4. Colocar breakpoints haciendo clic en el margen izquierdo de cualquier línea de código Dart.
5. Usar el panel **Flutter Inspector** para inspeccionar el árbol de widgets en vivo.

### Configurar la IP del broker MQTT en la app

Al abrir la app, tocar el ícono de configuración (⚙) en la barra superior e ingresar la IP del Raspberry Pi. Por defecto usa `192.168.1.100`.

---

## 5. Guía para el ingeniero electrónico — Construcción del robot

### Hardware requerido

| Componente | Descripción |
|---|---|
| Raspberry Pi 4 (o 3B+) | Computadora principal del robot |
| Driver L298N | Puente H para controlar 2 motores DC |
| 2x Motor DC con ruedas | Tracción diferencial |
| Módulo GPS NEO-6M | Posicionamiento con antena externa |
| Batería LiPo 7.4V o pack 18650 | Alimentación motores |
| Power bank 5V | Alimentación Raspberry Pi |
| Protoboard y cables | Conexiones |

### Diagrama de conexiones

#### L298N → Raspberry Pi

```
L298N        Raspberry Pi (GPIO BCM)
-------      ----------------------
IN1    →     GPIO 17
IN2    →     GPIO 27
IN3    →     GPIO 22
IN4    →     GPIO 23
ENA    →     GPIO 18 (PWM)
ENB    →     GPIO 24 (PWM)
GND    →     GND
5V out →     No conectar (usar alimentación separada para Pi)
```

#### NEO-6M GPS → Raspberry Pi

```
NEO-6M       Raspberry Pi
------       ------------
VCC    →     3.3V (pin 1)
GND    →     GND (pin 6)
TX     →     GPIO 15 / RXD (pin 10)
RX     →     GPIO 14 / TXD (pin 8)
```

Habilitar el puerto serial en el Pi:
```bash
sudo raspi-config
# Interfacing Options > Serial Port
# "Would you like a login shell accessible over serial?" → No
# "Would you like the serial port hardware to be enabled?" → Yes
```

### Software en la Raspberry Pi

#### 1. Instalar dependencias del sistema

```bash
sudo apt update
sudo apt install -y python3-pip mosquitto mosquitto-clients gpsd gpsd-clients
```

#### 2. Configurar Mosquitto (broker MQTT)

```bash
# Editar configuración para permitir conexiones externas
sudo nano /etc/mosquitto/mosquitto.conf
```

Contenido del archivo:

```
listener 1883
allow_anonymous true
```

```bash
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
sudo systemctl status mosquitto   # verificar que esté activo
```

#### 3. Instalar dependencias Python

```bash
pip3 install gpiozero paho-mqtt
```

#### 4. Configurar gpsd para el GPS

```bash
sudo nano /etc/default/gpsd
```

Contenido:

```
DEVICES="/dev/ttyS0"
GPSD_OPTIONS="-n"
START_DAEMON="true"
```

```bash
sudo systemctl enable gpsd
sudo systemctl start gpsd
cgps -s   # verificar lectura GPS (necesita cielo abierto ~30s)
```

#### 5. Script principal del robot

Crear el archivo `/home/pi/robot.py`:

```python
#!/usr/bin/env python3
import json
import math
import threading
import time
from gpiozero import Motor, PWMOutputDevice
import paho.mqtt.client as mqtt
import gpsd

# Pines GPIO
motor_izq = Motor(forward=17, backward=27)
motor_der = Motor(forward=22, backward=23)
pwm_izq   = PWMOutputDevice(18, initial_value=1)
pwm_der   = PWMOutputDevice(24, initial_value=1)

BROKER = "localhost"
mode   = "idle"
geofence_points = []

def set_speed(speed):
    pwm_izq.value = max(0.0, min(1.0, speed))
    pwm_der.value = max(0.0, min(1.0, speed))

def move(direction, speed):
    set_speed(speed)
    if direction == "fwd":
        motor_izq.forward(); motor_der.forward()
    elif direction == "back":
        motor_izq.backward(); motor_der.backward()
    elif direction == "left":
        motor_izq.backward(); motor_der.forward()
    elif direction == "right":
        motor_izq.forward(); motor_der.backward()
    else:
        motor_izq.stop(); motor_der.stop()

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Conectado al broker ({reason_code})")
    client.subscribe("robot/cmd/move", qos=0)
    client.subscribe("robot/cmd/mode", qos=1)
    client.subscribe("robot/cmd/geofence", qos=1)

def on_message(client, userdata, msg):
    global mode, geofence_points
    payload = json.loads(msg.payload.decode())

    if msg.topic == "robot/cmd/move" and mode == "manual":
        move(payload.get("dir", "stop"), payload.get("speed", 0.0))

    elif msg.topic == "robot/cmd/mode":
        mode = payload.get("mode", "idle")
        print(f"Modo: {mode}")
        if mode in ("stop", "idle"):
            move("stop", 0)
        status = {"mode": mode, "battery": get_battery(), "alert": "none"}
        client.publish("robot/status", json.dumps(status), qos=1)

    elif msg.topic == "robot/cmd/geofence":
        geofence_points = payload.get("points", [])
        print(f"Geofence recibido: {len(geofence_points)} puntos")

def get_battery():
    # Reemplazar con lectura real de ADC si se tiene sensor de batería
    return 85

def gps_loop(client):
    gpsd.connect()
    while True:
        try:
            packet = gpsd.get_current()
            if packet.mode >= 2:
                lat = packet.lat
                lng = packet.lon
                hdg = getattr(packet, 'track', 0.0) or 0.0
                position = {"lat": lat, "lng": lng, "hdg": hdg}
                client.publish("robot/position", json.dumps(position), qos=0)
        except Exception:
            pass
        time.sleep(1)

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="robot_pi")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, 1883, keepalive=60)

    gps_thread = threading.Thread(target=gps_loop, args=(client,), daemon=True)
    gps_thread.start()

    client.loop_forever()

if __name__ == "__main__":
    main()
```

#### 6. Ejecutar el robot al iniciar

```bash
# Crear servicio systemd
sudo nano /etc/systemd/system/robot.service
```

Contenido:

```ini
[Unit]
Description=Robot MQTT Controller
After=network.target mosquitto.service gpsd.service

[Service]
ExecStart=/usr/bin/python3 /home/pi/robot.py
WorkingDirectory=/home/pi
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable robot.service
sudo systemctl start robot.service
sudo systemctl status robot.service
```

### Verificar red WiFi

El robot y el celular deben estar en la **misma red WiFi**. Verificar la IP del Raspberry Pi:

```bash
hostname -I
# Ej: 192.168.1.100
```

Esa IP es la que se ingresa en la app al presionar el ícono de configuración.

---

## 6. Protocolo MQTT — Mensajes de la app y del robot

El broker MQTT corre en el Raspberry Pi en el puerto `1883`. Todos los mensajes usan payload **JSON**.

### Resumen de topics

| Topic | Dirección | QoS | Descripción |
|---|---|---|---|
| `robot/cmd/move` | App → Robot | 0 | Comando de movimiento (alta frecuencia) |
| `robot/cmd/mode` | App → Robot | 1 | Cambio de modo operativo |
| `robot/cmd/geofence` | App → Robot | 1 | Definición de zona geofence |
| `robot/position` | Robot → App | 0 | Posición GPS actual |
| `robot/status` | Robot → App | 1 | Estado del robot |

---

### Mensajes que envía la app al robot

#### `robot/cmd/move` — Comando de movimiento

Publicado a máximo **20 Hz** (cada 50ms) mientras el joystick está activo. QoS 0 porque la pérdida ocasional es tolerable a alta frecuencia.

```json
{
  "dir": "fwd",
  "speed": 0.75
}
```

| Campo | Tipo | Valores posibles | Descripción |
|---|---|---|---|
| `dir` | string | `fwd`, `back`, `left`, `right`, `stop` | Dirección de movimiento |
| `speed` | float | `0.0` a `1.0` | Intensidad (0 = parado, 1 = máxima velocidad) |

El robot **solo debe procesar este mensaje si está en modo `manual`**. Al recibir `dir: "stop"` o `speed: 0.0`, debe detener los motores.

---

#### `robot/cmd/mode` — Cambio de modo

Publicado con QoS 1 (garantizado). El robot debe responder publicando su nuevo estado en `robot/status`.

```json
{
  "mode": "manual"
}
```

| Campo | Tipo | Valores posibles | Descripción |
|---|---|---|---|
| `mode` | string | `manual`, `auto`, `stop` | Modo de operación solicitado |

- `manual`: el robot obedece comandos `robot/cmd/move` del joystick.
- `auto`: el robot navega de forma autónoma (lógica interna del Pi). Ignora comandos de movimiento.
- `stop`: el robot detiene motores inmediatamente y queda en espera.

---

#### `robot/cmd/geofence` — Zona geofence

Publicado con QoS 1 cuando el usuario dibuja un polígono en el mapa y presiona "Enviar". Contiene mínimo 3 puntos. El polígono es cerrado (el último punto se conecta con el primero).

```json
{
  "points": [
    { "lat": -25.2860, "lng": -57.6465 },
    { "lat": -25.2870, "lng": -57.6455 },
    { "lat": -25.2875, "lng": -57.6475 },
    { "lat": -25.2865, "lng": -57.6480 }
  ]
}
```

| Campo | Tipo | Descripción |
|---|---|---|
| `points` | array | Lista de vértices del polígono en orden |
| `points[n].lat` | float | Latitud decimal (WGS84) |
| `points[n].lng` | float | Longitud decimal (WGS84) |

El robot debe usar estos puntos para verificar si su posición GPS está dentro del polígono. Si sale, debe publicar `alert: "geofence_breach"` en `robot/status`.

---

### Mensajes que espera recibir la app desde el robot

#### `robot/position` — Posición GPS

Publicado por el robot periódicamente (recomendado cada 1 segundo). QoS 0.

```json
{
  "lat": -25.2867,
  "lng": -57.6470,
  "hdg": 135.5
}
```

| Campo | Tipo | Descripción |
|---|---|---|
| `lat` | float | Latitud decimal (WGS84) |
| `lng` | float | Longitud decimal (WGS84) |
| `hdg` | float | Rumbo en grados (0–360, Norte=0, Este=90) |

La app usa este mensaje para:
- Mover el marcador del robot en el mapa
- Rotar el ícono según el rumbo (`hdg`)
- Dibujar la traza del recorrido (últimas 200 posiciones)
- Centrar el mapa si el modo "seguir robot" está activo

---

#### `robot/status` — Estado del robot

Publicado por el robot al conectarse, al cambiar de modo, y periódicamente. QoS 1.

```json
{
  "mode": "manual",
  "battery": 78,
  "alert": "none"
}
```

| Campo | Tipo | Valores posibles | Descripción |
|---|---|---|---|
| `mode` | string | `idle`, `manual`, `auto`, `stop` | Modo actual del robot |
| `battery` | int | `0` a `100` | Porcentaje de batería |
| `alert` | string | `none`, `geofence_breach` | Alerta activa |

- `idle`: robot iniciado pero sin modo asignado aún.
- `geofence_breach`: el robot detectó que salió del polígono. La app resaltará el borde del polígono en rojo y mostrará una advertencia en la barra de estado.

---

### Flujo típico de una sesión

```
1. Robot inicia  →  publica robot/status {"mode":"idle", "battery":85, "alert":"none"}
2. App conecta al broker MQTT
3. App envía    →  robot/cmd/mode {"mode":"manual"}
4. Robot cambia a manual y publica robot/status {"mode":"manual", ...}
5. Usuario mueve joystick:
   App envía (20 Hz) → robot/cmd/move {"dir":"fwd", "speed":0.6}
6. Robot publica (1 Hz) → robot/position {"lat":..., "lng":..., "hdg":...}
7. Usuario dibuja zona en el mapa y presiona "Enviar":
   App envía → robot/cmd/geofence {"points":[...]}
8. App cambia a autónomo:
   App envía → robot/cmd/mode {"mode":"auto"}
9. Robot navega autónomamente. Si sale del geofence:
   Robot publica → robot/status {"mode":"auto", "battery":72, "alert":"geofence_breach"}
10. App muestra alerta visual en rojo
```

---

### Prueba sin hardware — Simulador

Para desarrollar y probar la app sin el robot físico, correr el simulador incluido en el repositorio:

```bash
# Instalar dependencias Python
pip3 install paho-mqtt

# Instalar y arrancar Mosquitto localmente
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto

# Correr el simulador (publica posición en círculo y responde comandos)
python3 tools/fake_robot.py
```

El simulador publica posiciones moviéndose en círculo alrededor del centro de Asunción y responde a los cambios de modo. En la app, configurar la IP del broker como `127.0.0.1` o la IP local de la PC.

---

## 7. Cómo probar la app

Guía rápida para levantar el rediseño (pantallas Inicio / Modo / Cerco / Actuadores) en un dispositivo y verificar que cada función siga funcionando contra el robot real o el simulador.

### Requisitos previos

| Requisito | Versión / detalle |
|---|---|
| Flutter | `3.44.0` (channel stable) |
| Dart | `3.12.0` (viene con el Flutter de arriba) |
| Android SDK | API 34 (Android 14), instalado según la [sección 3](#3-instalación-de-android-studio) |
| Dispositivo | Celular Android físico con depuración USB, **o** un emulador (AVD) con imagen API 34 |

Verificar versiones instaladas:

```bash
flutter --version
# Flutter 3.44.0 • ... • Dart 3.12.0
```

> Si vas a probar contra la Raspberry Pi real conectada por **hotspot del celular** (ver `docs/setup_mapa.md`), usá un teléfono físico: un emulador no puede unirse al hotspot porque no tiene radio WiFi real.

### Instalación de dependencias

```bash
cd mobile_app/lawnbot_app
flutter pub get
```

### Cómo correrla

**Opción A — modo debug con hot reload** (recomendado para revisar el rediseño pantalla por pantalla):

```bash
flutter devices        # confirmar que el celular/emulador aparece en la lista
flutter run             # o: flutter run -d <device-id> si hay más de uno
```

**Opción B — generar un APK de debug e instalarlo manualmente:**

```bash
flutter build apk --debug
adb install build/app/outputs/flutter-apk/app-debug.apk
```

### Requisitos de conectividad

1. El celular (o emulador) y la Raspberry Pi deben estar en la **misma red WiFi** (ver `docs/setup_wifi.md`).
2. Mosquitto en la Pi debe escuchar en todas las interfaces, no solo en `localhost`. Con `listener 1883` (sin IP) en `/etc/mosquitto/mosquitto.conf` ya escucha en `0.0.0.0:1883` — confirmarlo:
   ```bash
   sudo ss -tlnp | grep 1883
   # debe mostrar 0.0.0.0:1883, no 127.0.0.1:1883
   ```
3. Verificar los tres servicios de la Pi por SSH (ver `docs/setup_ssh.md`):
   ```bash
   ssh mow@<IP_DE_LA_PI>
   sudo systemctl status mosquitto
   sudo systemctl status bridge.service
   sudo systemctl status gpsd
   ```
   Los tres deben figurar `active (running)`.
4. Anotar la IP de la Pi (`hostname -I`) — es la que se ingresa tocando el ícono ⚙ en la barra superior de la app.
5. Este rediseño agrega el tópico `robot/cmd/actuator` y los campos `blade`/`trimmer`/`charging` a `robot/status`. La Pi necesita correr la versión actualizada de `bridge/bridge.py`; si quedó una versión vieja desplegada, la pantalla **Actuadores** no va a reflejar cambios (ver tabla de problemas comunes).

### Checklist funcional por pantalla

| Pantalla | Qué probar | Cómo verificar |
|---|---|---|
| **Inicio** | Alternar Joystick / Flechas y mover el robot; ver el robot y su recorrido en el mapa GPS en tiempo real | Al tocar el toggle Joystick/Flechas, `mosquitto_sub -h <IP_PI> -t robot/cmd/mode` debe mostrar `{"mode":"manual"}`; el marcador debe moverse y rotar según `hdg` |
| **Modo** | Tocar cada tarjeta: Líneas paralelas / Aleatorio / Perimetral | `mosquitto_sub -h <IP_PI> -t robot/cmd/mode` debe mostrar `auto_parallel`, `auto_random` o `auto_perimeter` al tocar cada una; la tarjeta seleccionada debe quedar resaltada |
| **Cerco** | Tocar el mapa para agregar puntos, deshacer, limpiar, enviar | El botón enviar solo se habilita con 3+ puntos; `mosquitto_sub -h <IP_PI> -t robot/cmd/geofence` debe mostrar el array de puntos al enviar |
| **Mapa GPS en tiempo real** | Mover la Pi (o caminar con ella) y ver el marcador desplazarse | Necesita `3D FIX` en el GPS (`cgps -s` en la Pi); la traza (polyline) se dibuja con el historial de posiciones |
| **Actuadores** | Switches de cuchilla y trimmer, carga, y "Parada de emergencia" | Cada switch solo cambia visualmente cuando la Pi confirma por `robot/status` (no es optimista); "Parada de emergencia" debe apagar ambos actuadores y dejar el modo en `stop` |

### Problemas comunes

| Problema | Causa probable | Solución |
|---|---|---|
| La app no conecta al broker | Celular y Pi en redes distintas, o Mosquitto solo escucha en `localhost` | Confirmar misma WiFi; revisar `listener 1883` en `mosquitto.conf` y que `ss -tlnp` muestre `0.0.0.0:1883` |
| El GPS no actualiza en el mapa | Sin fix GPS (el NEO-6M necesita cielo abierto, ~30s a 5 min) o `gpsd` caído | `cgps -s` en la Pi hasta ver `3D FIX`; `sudo systemctl status gpsd` |
| El joystick/flechas no mueve el robot | El robot no está en modo `manual` | Revisar `robot/status` con `mosquitto_sub`; volver a tocar el toggle Joystick/Flechas (publica `{"mode":"manual"}`) |
| Los switches de Actuadores no cambian nunca | La Pi corre una versión vieja de `bridge.py` sin el tópico `robot/cmd/actuator` | En la Pi: `git pull` en el repo, `sudo systemctl restart bridge.service` |
| No llega nada en `robot/position` | `bridge.service` caído o gpsd sin fix | `sudo systemctl restart bridge.service`; logs con `journalctl -u bridge.service -f` |
| Cambié de modo pero la UI no lo refleja | El robot no devolvió `robot/status` | Revisar logs de `bridge.service`; confirmar que el `mode` recibido es uno válido (`manual`, `stop`, `auto_parallel`, `auto_random`, `auto_perimeter`) |

### Probar sin hardware

Si no tenés la Raspberry Pi a mano, seguí la [sección "Prueba sin hardware"](#prueba-sin-hardware--simulador) para levantar `tools/fake_robot.py` con Mosquitto local — el simulador ya soporta el tópico `robot/cmd/actuator` y confirma los estados de cuchilla/trimmer/carga igual que lo haría la Pi real.
