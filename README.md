# Lawn Mower Robot 🌿

Robot móvil cortacésped autónomo — Trabajo Práctico Final Robótica II  
Ing. Mecatrónica · FIUNA · 2025S2

## Stack tecnológico

| Capa | Tecnología |
|---|---|
| Navegación / Control | ROS2 (Raspberry Pi 4) |
| Visión artificial | YOLOv8n-seg + NCNN (inferencia CPU) |
| Hardware embebido | ESP32 + L298N |
| App móvil | Flutter 3 + MQTT (Mosquitto) |
| Comunicación | Serial UART RPi → ESP32, WiFi app → RPi |

## Estructura del repositorio

```
lawn-mower-robot/
├── ros2_ws/                  # Workspace ROS2 principal
│   └── src/
│       ├── lawnbot_bringup/      # Launch files
│       ├── lawnbot_hardware/     # Drivers motores, sensores
│       ├── lawnbot_navigation/   # Líneas paralelas, aleatorio, perimetral
│       ├── lawnbot_vision/       # Segmentación + campo de potencial
│       ├── lawnbot_localization/ # GPS, IMU, odometría, mapa
│       └── lawnbot_msgs/         # Mensajes y servicios custom
├── firmware/
│   ├── esp32_motor_control/  # PWM, L298N, encoders
│   └── esp32_sensors/        # Bumpers, US, buzzer, batería
├── mobile_app/
│   └── lawnbot_app/          # Proyecto Flutter
├── vision/
│   ├── training/             # Notebooks Colab, configs YOLOv8
│   ├── models/               # Pesos .pt / .ncnn (ver Drive)
│   └── dataset_tools/        # Scripts remapeo clases
├── bridge/                   # bridge.py: MQTT ↔ ROS2 ↔ ESP32
├── docs/                     # Informes, diagramas, fotos
└── tests/                    # Pruebas unitarias por módulo
```

## Setup rápido

```bash
# Clonar
git clone git@github.com:carolparedesf/lawn-mower-robot.git
cd lawn-mower-robot

# Build ROS2
cd ros2_ws
colcon build
source install/setup.bash

# Lanzar robot completo
ros2 launch lawnbot_bringup lawnbot.launch.py

# Bridge app ↔ robot
python3 bridge/bridge.py
```

## Clases de segmentación (visión)

`grass · tree · plant · ground · path · obstacle`  
Modelo: YOLOv8n-seg exportado a NCNN — Mask mAP50 = 0.486 (V1)

## Autores

- Carol Paredes — [@carolparedesf](https://github.com/carolparedesf)

FIUNA · Universidad Nacional de Asunción
