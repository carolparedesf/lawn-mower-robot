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
