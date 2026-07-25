#!/usr/bin/env python3
"""Simulador de robot para pruebas sin hardware."""

import json
import math
import time
import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883

# Centro de simulación (Asunción, Paraguay)
CENTER_LAT = -25.2867
CENTER_LNG = -57.6470
RADIUS_DEG = 0.0005  # ~55 metros

mode = "idle"
battery = 85
angle = 0.0
blade_on = False
trimmer_on = False
charging = False


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Conectado al broker ({reason_code})")
    client.subscribe("robot/cmd/mode")
    client.subscribe("robot/cmd/geofence")
    client.subscribe("robot/cmd/move")
    client.subscribe("robot/cmd/actuator")


def publish_status(client):
    status = {
        "mode": mode,
        "battery": battery,
        "alert": "none",
        "blade": blade_on,
        "trimmer": trimmer_on,
        "charging": charging,
    }
    client.publish("robot/status", json.dumps(status), qos=1)


def on_message(client, userdata, msg):
    global mode, blade_on, trimmer_on, charging
    payload = json.loads(msg.payload.decode())
    if msg.topic == "robot/cmd/mode":
        mode = payload.get("mode", mode)
        print(f"[CMD] Modo cambiado a: {mode}")
        publish_status(client)
    elif msg.topic == "robot/cmd/geofence":
        points = payload.get("points", [])
        print(f"[CMD] Geofence recibido con {len(points)} puntos")
    elif msg.topic == "robot/cmd/move":
        direction = payload.get("dir", "stop")
        speed = payload.get("speed", 0.0)
        print(f"[CMD] Move: dir={direction} speed={speed:.2f}")
    elif msg.topic == "robot/cmd/actuator":
        blade_on = payload.get("blade", False)
        trimmer_on = payload.get("trimmer", False)
        charging = payload.get("charging", False)
        print(f"[CMD] Actuadores: blade={blade_on} trimmer={trimmer_on} charging={charging}")
        publish_status(client)


def main():
    global angle, battery

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake_robot")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    print("Simulador iniciado. Publicando posición cada 2 segundos...")
    tick = 0

    while True:
        angle = (angle + 5) % 360
        rad = math.radians(angle)
        lat = CENTER_LAT + RADIUS_DEG * math.sin(rad)
        lng = CENTER_LNG + RADIUS_DEG * math.cos(rad)
        hdg = angle

        position = {"lat": lat, "lng": lng, "hdg": hdg}
        client.publish("robot/position", json.dumps(position), qos=0)

        if tick % 10 == 0 and tick > 0:
            battery = min(100, battery + 2) if charging else max(0, battery - 1)
            publish_status(client)

        tick += 1
        time.sleep(2)


if __name__ == "__main__":
    main()