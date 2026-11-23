import argparse
import time
from collections import deque
import cv2
import numpy as np
from ultralytics import YOLO

ap = argparse.ArgumentParser()
ap.add_argument("--model", default="ncnn_640")
ap.add_argument("--source", default="0", help="0 = camara, o ruta a un video")
ap.add_argument("--imgsz", type=int, default=640, help="debe coincidir con el export")
ap.add_argument("--conf", type=float, default=0.4)
ap.add_argument("--min-grass", type=float, default=0.10)
ap.add_argument("--show", action="store_true", help="mostrar ventana")
ap.add_argument("--save", default="", help="ruta para guardar video con resultados")
args = ap.parse_args()

def base_point(pts):
    ys = pts[:, 1]
    ymax = ys.max()
    xs = pts[ys >= ymax - 2][:, 0]
    return int(xs.mean()), int(ymax)

def centroid(pts):
    m = cv2.moments(pts.reshape(-1, 1, 2))
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

model = YOLO(args.model, task="segment")
print("Clases del modelo:", model.names)   # comparar con data.yaml

src = int(args.source) if args.source.isdigit() else args.source
cap = cv2.VideoCapture(src)
writer = None
hist = deque(maxlen=7)
t0 = time.time()
frames = 0

while True:
    ok, frame = cap.read()
    if not ok:
        break
    h, w = frame.shape[:2]
    r = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
    grass_area = 0.0
    trees, plants = [], []
    if r.masks is not None:
        for i, pts in enumerate(r.masks.xy):
            if len(pts) < 3:
                continue
            pts = pts.astype(np.int32)
            name = model.names[int(r.boxes.cls[i])]
            if name == "pasto":
                grass_area += cv2.contourArea(pts.reshape(-1, 1, 2))
            elif name == "arbol":
                trees.append(base_point(pts))
            elif name == "planta":
                plants.append((base_point(pts), centroid(pts)))
    hist.append(grass_area / (w * h) >= args.min_grass)
    cut_on = sum(hist) > len(hist) / 2
    frames += 1
    fps = frames / (time.time() - t0)
    print(f"FPS {fps:4.1f} | pasto {grass_area/(w*h):.2f} | corte {'ON' if cut_on else 'OFF'} "
          f"| arboles {trees} | plantas {plants}"f"| v {v:.2f} m/s | w {wz:+.2f} rad/s")
    if args.show or args.save:
        vis = r.plot()
        for bx, by in trees:
            cv2.circle(vis, (bx, by), 8, (0, 0, 255), -1)
        for (bx, by), c in plants:
            cv2.circle(vis, (bx, by), 8, (0, 0, 255), -1)
            if c:
                cv2.circle(vis, c, 8, (255, 0, 0), -1)
        cv2.putText(vis, "CORTE ON" if cut_on else "CORTE OFF", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0) if cut_on else (0, 0, 255), 3)
        if args.save and writer is None:
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 10, (w, h))
        if writer is not None:
            writer.write(vis)
        if args.show:
            cv2.imshow("lawnbot", vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

cap.release()
if writer is not None:
    writer.release()
cv2.destroyAllWindows()
