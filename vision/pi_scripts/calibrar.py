import sys
import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO(sys.argv[1] if len(sys.argv) > 1 else "yolov8n-seg.pt")
cap = cv2.VideoCapture(0)
distancias = [0.5, 1.0, 1.5, 2.0]
resultados = []

for d in distancias:
    input(f"Pon el objeto con su BASE sobre la marca de {d} m y presiona Enter... ")
    for _ in range(10):
        cap.read()                      # descarta cuadros viejos
    ys = []
    for _ in range(8):
        ok, f = cap.read()
        r = model.predict(f, conf=0.4, imgsz=320, verbose=False)[0]
        if r.masks is None or len(r.masks.xy) == 0:
            continue
        mejor = max(r.masks.xy, key=lambda p: cv2.contourArea(p.astype(np.int32).reshape(-1, 1, 2)))
        ys.append(float(mejor[:, 1].max()))
    if ys:
        y = float(np.median(ys))
        resultados.append((y, d))
        print(f"   d = {d} m  ->  y_base = {y:.0f}   (alto de imagen = {f.shape[0]})")
    else:
        print("   no detecté nada, repite este punto")

resultados.sort()
print()
print("CALIB_Y =", [round(y) for y, d in resultados])
print("CALIB_D =", [d for y, d in resultados])
ds = [d for y, d in resultados]
if ds != sorted(ds, reverse=True):
    print("AVISO: las distancias no bajan al subir y_base; repite las mediciones.")
EOF
