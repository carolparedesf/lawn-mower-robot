import argparse
import threading
import time
import cv2
from flask import Flask, Response
from ultralytics import YOLO
from vision_core import Pipeline

ap = argparse.ArgumentParser()
ap.add_argument("--model", default="ncnn_640")
ap.add_argument("--source", default="0")
ap.add_argument("--imgsz", type=int, default=640)
ap.add_argument("--conf", type=float, default=0.4)
ap.add_argument("--min-grass", type=float, default=0.10)
ap.add_argument("--port", type=int, default=5000)
args = ap.parse_args()

app = Flask(__name__)
latest = {"jpg": None}

def worker():
    model = YOLO(args.model, task="segment")
    print("Clases del modelo:", model.names)
    pipe = Pipeline(model, args.conf, args.imgsz, args.min_grass)
    is_file = not args.source.isdigit()
    cap = cv2.VideoCapture(args.source if is_file else int(args.source))
    t0 = time.time()
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            if is_file:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)   # repite el video
                continue
            break
        vis, info = pipe.process(frame)
        n += 1
        fps = n / (time.time() - t0)
        cv2.putText(vis, f"FPS {fps:.1f}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        ok2, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok2:
            latest["jpg"] = buf.tobytes()
        print(f"FPS {fps:4.1f} | corte {'ON' if info['cut_on'] else 'OFF'} | "
              f"v {info['v']:.2f} w {info['w']:+.2f} | obstaculos {info['n_obst']}")

def gen():
    while True:
        jpg = latest["jpg"]
        if jpg is None:
            time.sleep(0.05)
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        time.sleep(0.05)

@app.route("/")
def index():
    return '<html><body style="margin:0;background:#111"><img src="/video" style="width:100%"></body></html>'

@app.route("/video")
def video():
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

threading.Thread(target=worker, daemon=True).start()
app.run(host="0.0.0.0", port=args.port, threaded=True)
