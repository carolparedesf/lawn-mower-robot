# Puertos USB fijos en la Raspberry Pi

En la Pi, el ESP32 y el TF-Mini aparecen como `/dev/ttyUSB0` y `/dev/ttyUSB1`, y el orden puede
cambiar entre arranques. Para evitarlo se instalan reglas `udev` que crean nombres fijos.

| Dispositivo | Chip USB | ID (vendor:producto) | Nombre fijo |
|---|---|---|---|
| ESP32 (micro-ROS) | CH340 | `1a86:7523` | `/dev/esp32` |
| TF-Mini Plus | CP2102 | `10c4:ea60` | `/dev/tfmini` |

El GPS no usa USB: gpsd lee `/dev/ttyAMA0` (UART de la Pi).

## Instalar (en la Pi)

Desde la raíz del repo clonado en la Pi (o copiando el archivo con `scp`):

```bash
sudo cp docs/hardware/99-lawnbot-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/esp32 /dev/tfmini
```

Cada enlace debe apuntar a un `ttyUSB*`. Si no aparece, desenchufar y volver a enchufar el dispositivo.

## Usar los nombres fijos

```bash
# micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 921600

# TF-Mini (el nodo tiene un parámetro `port`; por defecto abre /dev/ttyUSB0)
python3 tfMiniNode.py --ros-args -p port:=/dev/tfmini
```

## Verificar cuál es cuál

El TF-Mini (115200 baudios) manda tramas que empiezan con `59 59`:

```bash
python3 -c "import serial; s=serial.Serial('/dev/tfmini',115200,timeout=2); print(s.read(27).hex())"
```

Si aparece `5959...`, `/dev/tfmini` es el correcto.

## Notas

- Las reglas se basan en vendor/producto, no en el puerto físico. Si se conecta otro dispositivo
  con el mismo chip (otro CP2102 o CH340), compartiría el nombre. Con dos dispositivos del mismo tipo
  hay que distinguirlos por puerto físico (`ATTRS{devpath}` o `KERNELS`).
- Al flashear desde la laptop, el ESP32 puede aparecer como `/dev/ttyACM0`. Estas reglas solo
  aplican en la Pi.
