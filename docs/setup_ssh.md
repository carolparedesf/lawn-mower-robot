# Conexión a la Raspberry Pi por SSH

## Requisitos
- Pi y computadora en la misma red WiFi o hotspot
- SSH habilitado en la Pi

## Conectarse
```bash
ssh mow@IP_DE_LA_PI
```

Para saber la IP de la Pi:
```bash
hostname -I
```

## Primera conexión
La primera vez pregunta si confiás en el host, escribí `yes` y presionás Enter.

## Ejemplo
```bash
ssh mow@10.183.212.234
```

## Comandos útiles una vez conectado

| Comando | Descripción |
|---|---|
| `sudo systemctl status bridge.service` | Ver estado del bridge |
| `sudo systemctl restart bridge.service` | Reiniciar bridge |
| `sudo systemctl status mosquitto` | Ver estado del broker |
| `mosquitto_sub -h localhost -t "robot/position"` | Ver posición en tiempo real |
| `cgps -s` | Ver estado del GPS |
| `hostname -I` | Ver IP de la Pi |

## Abrir múltiples terminales SSH
Podés abrir varias ventanas de terminal y conectarte a la Pi en cada una para correr distintos comandos simultáneamente.
