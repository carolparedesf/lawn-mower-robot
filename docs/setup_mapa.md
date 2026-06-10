# Instructivo: Visualización del Robot en el Mapa

## Requisitos previos
- Raspberry Pi con Ubuntu 22.04
- BerryIMUgps conectado
- App instalada en el celular
- Mosquitto y bridge.py configurados

---

## Paso 1 — Activar hotspot en el celular
1. Abrí configuración del celular
2. Activá el hotspot personal
3. Anotá el nombre y contraseña

---

## Paso 2 — Conectar la Pi al hotspot
Desde la Pi:
```bash
sudo nmcli device wifi connect "NOMBRE_HOTSPOT" password "CONTRASEÑA"
```
Verificá la IP asignada:
```bash
hostname -I
```

---

## Paso 3 — Verificar servicios en la Pi
```bash
sudo systemctl status mosquitto
sudo systemctl status bridge.service
sudo systemctl status gpsd
```
Los tres deben estar `active (running)`.

---

## Paso 4 — Verificar señal GPS
```bash
cgps -s
```
Esperá afuera con cielo abierto hasta ver `3D FIX` y coordenadas reales. Puede tardar 1-5 minutos.

---

## Paso 5 — Verificar publicación de posición
```bash
mosquitto_sub -h localhost -t "robot/position"
```
Debe aparecer JSON cada segundo:
```json
{"lat": -25.xxxx, "lng": -57.xxxx, "hdg": 0.0}
```

---

## Paso 6 — Conectar la app
1. Abrí la app en el celular
2. Ingresá la IP de la Pi
3. Tocá conectar
4. Andá a la pantalla del mapa

---

## Paso 7 — Ver el robot en el mapa
- El ícono azul aparece en tu ubicación real
- Rota según el heading del GPS
- Se mueve en tiempo real mientras caminás
- La línea azul muestra el recorrido

---

## Solución de problemas

| Problema | Solución |
|---|---|
| `NO FIX` en GPS | Salir afuera, esperar 5 min |
| App no conecta | Verificar IP y que estén en la misma red |
| No aparece JSON | `sudo systemctl restart bridge.service` |
| Mosquitto no arranca | `sudo pkill mosquitto && sudo systemctl start mosquitto` |
